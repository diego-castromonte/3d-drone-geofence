import time
import math
import random
import socket
import json
from pymavlink import mavutil

# =====================================================================
# 1. SAFETY & BOUNDARY CONFIGURATION (Local NED Frame)
X_GEOFENCE_LIMIT = 25.0  # North/South boundary (+/- 25m)
Y_GEOFENCE_LIMIT = 25.0  # East/West boundary (+/- 25m)
Z_MAX_ALTITUDE = -20.0   # Ceiling: 20m high (NED Z is negative upward)
Z_MIN_ALTITUDE = -3.0    # Floor: 3m high

SAFE_ALTITUDE = -10.0    # Altitude target when recovering back inside (10m high)
SAFE_RADIUS = 10.0       # Inner boundary threshold for recovery completion
MIN_INTER_DRONE_DIST = 4.0 # Minimum safe separation distance between vehicles

KP_GAIN = 1.3            # Proportional gain for velocity control scaling
MAX_SPEED_HORIZ = 6.0    # Max horizontal velocity target (m/s)
MAX_SPEED_VERT = 1.5     # Max vertical ascent/descent velocity target (m/s)

MASK_POSITION = 0b110111111000  # Position Control Bitmask
MASK_VELOCITY = 0b110111000111  # Velocity Control Bitmask (VX, VY, VZ)

# =====================================================================
# 2. SYNCHRONIZED TANGO ORBIT & UDP BROADCAST CONFIGURATION
ORBIT_RADIUS = 8.0          # Circle radius in meters (scaled for 3 drones)
ORBIT_ALTITUDE = -10.0      # Cruise altitude during orbit (-10m = 10m high)
ORBIT_SPEED = 0.05          # Angular stepping speed per tick (radians per 0.1s)
TOTAL_ROTATIONS = 2         # Number of full 360-degree rotations to perform

NUM_DRONES = 3              # Total active vehicles in simulation

# --- UDP BROADCAST SOCKET FOR MATLAB DASHBOARD (PORT 5005) ---
matlab_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
matlab_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
MATLAB_ADDR = ("<broadcast>", 5005)

last_print_time = 0.0

# =====================================================================
# 3. HELPER FUNCTIONS
def generate_waypoints(count=3):
    return [
        (round(random.uniform(-35, 35), 2), 
         round(random.uniform(-35, 35), 2), 
         round(random.uniform(-25, -1), 2))
        for _ in range(count)
    ]

def arm_and_takeoff(drone, target_alt):
    print(f"[Drone {drone.target_system}] Setting GUIDED mode...")
    drone.set_mode('GUIDED')
    
    print(f"[Drone {drone.target_system}] Arming motors...")
    drone.arducopter_arm()
    drone.motors_armed_wait()
    
    print(f"[Drone {drone.target_system}] Taking off to {target_alt}m...")
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_alt
    )
    time.sleep(6)

def send_velocity_cmd(drone, vx, vy, vz):
    drone.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, drone.target_system, drone.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASK_VELOCITY,
            0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0
        )
    )

def send_position_cmd(drone, x, y, z):
    drone.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, drone.target_system, drone.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, MASK_POSITION,
            x, y, z, 0, 0, 0, 0, 0, 0, 0, 0
        )
    )

def get_local_telemetry(drone):
    msg = drone.recv_match(type='LOCAL_POSITION_NED', blocking=False)
    if msg:
        return (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
    return None

def check_inter_drone_distance(pos_a, pos_b):
    if not pos_a or not pos_b:
        return 999.0 
    return math.sqrt((pos_a[0]-pos_b[0])**2 + (pos_a[1]-pos_b[1])**2 + (pos_a[2]-pos_b[2])**2)

def log_status(sys_id, state, cx, cy, cz, target, tvx=0.0, tvy=0.0, tvz=0.0):
    if state in ["FLYING", "RECOVERING"]:
        print(f"[Drone {sys_id} - {state}] Pos:({cx:.1f}, {cy:.1f}, {cz:.1f})m -> "
              f"Target:({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})m | "
              f"Cmd Vel:({tvx:.2f}, {tvy:.2f}, {tvz:.2f})m/s")
    else:
        print(f"[Drone {sys_id} - {state}] Pos:({cx:.1f}, {cy:.1f}, {cz:.1f})m -> "
              f"Cmd Pos:({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})m")

# =====================================================================
# 4. CONNECTION & DYNAMIC 3-DRONE INITIALIZATION
connections = [
    ('udpin:localhost:14551', 1),
    ('udpin:localhost:14561', 2),
    ('udpin:localhost:14571', 3)
]

drones = {}

for addr, sys_id in connections:
    print(f"Connecting to Drone {sys_id} on {addr}...")
    conn = mavutil.mavlink_connection(addr)
    conn.wait_heartbeat()
    arm_and_takeoff(conn, 5)

    gate_angle = (sys_id - 1) * (2 * math.pi / NUM_DRONES)
    entry_gate = (ORBIT_RADIUS * math.cos(gate_angle), ORBIT_RADIUS * math.sin(gate_angle), ORBIT_ALTITUDE)

    drones[sys_id] = {
        'conn': conn,
        'state': "FLYING",
        'waypoints': generate_waypoints(3) + [entry_gate],
        'wp_idx': 0,
        'hold_pos': (0.0, 0.0, -5.0),
        'cx': 0.0, 'cy': 0.0, 'cz': -5.0
    }

theta = 0.0 
max_theta = TOTAL_ROTATIONS * 2 * math.pi

print("\nAll 3 Drones Connected, Armed, and Ready. Starting Loop...\n")

# =====================================================================
# 5. CLOSED-LOOP STATE MACHINE & MULTI-VEHICLE CONTROL LOOP
try:
    while True:
        current_time = time.time()
        should_print = (current_time - last_print_time) >= 2.0

        all_ready = all(d['state'] == "READY_FOR_TANGO" for d in drones.values())

        if all_ready:
            if should_print:
                print("\n*** ALL 3 DRONES IN POSITION: STARTING TRIANGLE TANGO ***\n")
            for d in drones.values():
                d['state'] = "TANGO"

        if drones[1]['state'] == "TANGO":
            theta += ORBIT_SPEED
            if theta >= max_theta:
                print("\n*** TRIANGLE TANGO COMPLETE -- SWITCHING TO POSITION HOLD *** \n")
                for d in drones.values():
                    d['state'] = "HOLDING"
                    d['hold_pos'] = (d['cx'], d['cy'], d['cz'])

        for sys_id, d in drones.items():
            conn = d['conn']
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

            telem = get_local_telemetry(conn)
            if not telem:
                continue

            d['cx'], d['cy'], d['cz'], cvx, cvy, cvz = telem
            cx, cy, cz = d['cx'], d['cy'], d['cz']
            curr_wp = d['waypoints'][d['wp_idx']]

            x_breach = abs(cx) >= X_GEOFENCE_LIMIT
            y_breach = abs(cy) >= Y_GEOFENCE_LIMIT
            z_breach = (cz <= Z_MAX_ALTITUDE) or (cz >= Z_MIN_ALTITUDE)

            if (x_breach or y_breach or z_breach) and d['state'] == "FLYING":
                print(f"\n*** [Drone {sys_id}] GEOFENCE BREACH at ({cx:.2f}, {cy:.2f}, {cz:.2f})m! ***")
                d['state'] = "RECOVERING"

            if d['state'] == "FLYING":
                dx, dy, dz = curr_wp[0] - cx, curr_wp[1] - cy, curr_wp[2] - cz
                dist_to_wp = math.sqrt(dx**2 + dy**2 + dz**2)

                dist_x_wall = X_GEOFENCE_LIMIT - abs(cx)
                dist_y_wall = Y_GEOFENCE_LIMIT - abs(cy)
                dist_z_wall = min(abs(Z_MAX_ALTITUDE - cz), abs(Z_MIN_ALTITUDE - cz))
                min_wall_dist = min(dist_x_wall, dist_y_wall, dist_z_wall)

                SLOWDOWN_ZONE = 8.0
                fence_speed = max(0.15, min_wall_dist / SLOWDOWN_ZONE) if min_wall_dist < SLOWDOWN_ZONE else 1.0

                max_h = MAX_SPEED_HORIZ * fence_speed
                max_v = MAX_SPEED_VERT * fence_speed

                tvx = max(-max_h, min(max_h, KP_GAIN * dx))
                tvy = max(-max_h, min(max_h, KP_GAIN * dy))
                tvz = max(-max_v, min(max_v, KP_GAIN * dz))

                send_velocity_cmd(conn, tvx, tvy, tvz)

                if should_print:
                    log_status(sys_id, "FLYING", cx, cy, cz, curr_wp, tvx, tvy, tvz)

                if dist_to_wp <= 1.5:
                    print(f"\n>>> [Drone {sys_id}] Reached WP{d['wp_idx']+1} at ({cx:.2f}, {cy:.2f}, {cz:.2f})m\n")
                    if d['wp_idx'] + 1 < len(d['waypoints']):
                        d['wp_idx'] += 1
                    else:
                        d['state'] = "READY_FOR_TANGO"

            elif d['state'] == "RECOVERING":
                dx, dy, dz = 0.0 - cx, 0.0 - cy, SAFE_ALTITUDE - cz
                tvx = max(-5.0, min(5.0, KP_GAIN * dx))
                tvy = max(-5.0, min(5.0, KP_GAIN * dy))
                tvz = max(-2.0, min(2.0, KP_GAIN * dz))

                send_velocity_cmd(conn, tvx, tvy, tvz)

                if should_print:
                    log_status(sys_id, "RECOVERING", cx, cy, cz, (0.0, 0.0, SAFE_ALTITUDE), tvx, tvy, tvz)

                in_xy_safe = abs(cx) <= SAFE_RADIUS and abs(cy) <= SAFE_RADIUS
                in_z_safe = Z_MAX_ALTITUDE <= cz <= Z_MIN_ALTITUDE
                curr_speed = math.sqrt(cvx**2 + cvy**2 + cvz**2)

                if in_xy_safe and in_z_safe and curr_speed <= 0.2:
                    if d['wp_idx'] + 1 < len(d['waypoints']):
                        d['wp_idx'] += 1
                        d['state'] = "FLYING"
                    else:
                        d['hold_pos'] = (cx, cy, cz)
                        d['state'] = "HOLDING"

            elif d['state'] == "READY_FOR_TANGO":
                target_entry = d['waypoints'][-1]
                send_position_cmd(conn, *target_entry)
                if should_print:
                    log_status(sys_id, "READY_FOR_TANGO", cx, cy, cz, target_entry)

            elif d['state'] == "TANGO":
                angle = theta + (sys_id - 1) * (2 * math.pi / NUM_DRONES)
                target_x = ORBIT_RADIUS * math.cos(angle)
                target_y = ORBIT_RADIUS * math.sin(angle)
                target_z = ORBIT_ALTITUDE 

                send_position_cmd(conn, target_x, target_y, target_z)

                if should_print:
                    log_status(sys_id, "TANGO", cx, cy, cz, (target_x, target_y, target_z))

            elif d['state'] == "HOLDING":
                send_position_cmd(conn, *d['hold_pos'])
                if should_print:
                    log_status(sys_id, "HOLDING", cx, cy, cz, d['hold_pos'])

        # Collision avoidance evaluation
        for i in range(1, NUM_DRONES + 1):
            for j in range(i + 1, NUM_DRONES + 1):
                p1 = (drones[i]['cx'], drones[i]['cy'], drones[i]['cz'])
                p2 = (drones[j]['cx'], drones[j]['cy'], drones[j]['cz'])
                dist = check_inter_drone_distance(p1, p2)

                if dist < MIN_INTER_DRONE_DIST:
                    if should_print:
                        print(f"[WARNING] PROXIMITY ALERT between Drone {i} and {j}! Dist: {dist:.2f}m")
                    send_velocity_cmd(drones[i]['conn'], 1.5, 0, 0)
                    send_velocity_cmd(drones[j]['conn'], -1.5, 0, 0)

        # =====================================================================
        # 6. UDP TELEMETRY BROADCAST (SEND PACKETS TO MATLAB PORT 5005)
        telem_packet = {
            f"d{sys_id}": [d['cx'], d['cy'], d['cz']] for sys_id, d in drones.items()
        }
        matlab_socket.sendto(json.dumps(telem_packet).encode(), MATLAB_ADDR)

        if should_print:
            last_print_time = current_time

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping loop.")