"""
Virtual Safety Boundary Guard & Emergency Disturbance Recovery
----------------------------------------------------------------
Closed-loop 3D geofence safety controller for ArduCopter (SITL).

Continuously monitors LOCAL_POSITION_NED telemetry over MAVLink, enforces a
3D geofence box, and autonomously recovers the vehicle via a proportional
velocity controller when a breach is detected. Adaptive proximity slowdown
throttles cruise speed near the fence walls as a second, softer safety
layer independent of hard breach recovery.

State machine: FLYING -> RECOVERING -> HOLDING
See README.md for the full write-up, control equations, and a sample run.
"""

import time
import math
import random
from pymavlink import mavutil

# 1. CONNECT TO SITL & INITIALIZE VEHICLE
# Open UDP connection to ArduPilot SITL instance on local port 14551
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

# 3D GEOFENCE BOX BOUNDARIES (Local NED Frame)
X_GEOFENCE_LIMIT = 25.0          # Hard boundary for X (North/South) in meters (+/- 25m)
Y_GEOFENCE_LIMIT = 25.0          # Hard boundary for Y (East/West) in meters (+/- 25m)
Z_MAX_ALTITUDE = -20.0           # Hard ceiling for Z in meters (20m up; Z is negative upward!)
Z_MIN_ALTITUDE = -3.0            # Hard floor for Z in meters (3m up)

# RECOVERY TARGETS
SAFE_INSIDE_TARGET = 15.0        # Boundary distance threshold for recovery
SAFE_ALTITUDE = -10.0            # Safe recovery altitude center (10m high)
SAFE_RADIUS = 10.0               # Safe radius inside fence for emergency recovery completion

# CONTROLLER GAINS
KP_GAIN = 1.3                   # Proportional gain scaling target speed
MAX_SPEED_HORIZ = 6.0            # Maximum horizontal velocity (m/s)
MAX_SPEED_VERT = 1.5             # Maximum vertical velocity (m/s)

# MAVLINK BITMASKS (SET_POSITION_TARGET_LOCAL_NED)
mask_position = 0b110111111000   # Enables Position Control (used for stationary position hold)
mask_velocity = 0b110111000111   # Enables Velocity Control (used for streaming VX, VY, VZ)

# Initialize hold coordinates for emergency stop / completion
hold_x = 0.0
hold_y = 0.0
hold_z = -10.0

# =====================================================================
# DYNAMIC 3D WAYPOINT GENERATION
# Generate random (X, Y, Z) targets between -35m and +35m.
waypoints = [
    (round(random.uniform(-35, 35), 2),
     round(random.uniform(-35, 35), 2),
     round(random.uniform(-25, -1), 2))  # Z target (negative upward)
    for _ in range(3)
]
current_wp_index = 0
print(f"Generated Mission Queue: {waypoints}")

# =====================================================================
# 2. ARM & TAKEOFF SEQUENCE
print("Arming vehicle...")
the_connection.mav.command_long_send(
    the_connection.target_system, the_connection.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
)
time.sleep(1)

print("Taking off to 10m...")
the_connection.mav.command_long_send(
    the_connection.target_system, the_connection.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10
)
time.sleep(8)  # Wait for quadcopter to climb to altitude
state = "FLYING"

# =====================================================================
# 3. CLOSED-LOOP CONTROL LOOP
while 1:
    # Read real-time telemetry from SITL
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    if not msg:
        continue

    current_x = msg.x
    current_y = msg.y
    current_z = msg.z
    current_vx = msg.vx  # Current velocity in X direction
    current_vy = msg.vy  # Current velocity in Y direction
    current_vz = msg.vz  # Current velocity in Z direction

    target_wp = waypoints[current_wp_index]

    # -----------------------------------------------------------------
    # Using abs() checks both positive and negative directions simultaneously!
    x_breach = abs(current_x) >= X_GEOFENCE_LIMIT
    y_breach = abs(current_y) >= Y_GEOFENCE_LIMIT
    z_breach = (current_z <= Z_MAX_ALTITUDE) or (current_z >= Z_MIN_ALTITUDE)

    if (x_breach or y_breach or z_breach) and state == "FLYING":
        print(f"\n*** 3D GEOFENCE BREACH AT Pos:({current_x:.2f}, {current_y:.2f}, {current_z:.2f})m! ***\n")
        print(f"Aborting invalid waypoint {target_wp} and executing recovery...\n")
        state = "RECOVERING"

    # -----------------------------------------------------------------
    # STATE MACHINE & 3D VECTOR CALCULATIONS
    if state == "FLYING":
        # Calculate 3D distance components to current target waypoint
        dx = target_wp[0] - current_x
        dy = target_wp[1] - current_y
        dz = target_wp[2] - current_z
        # Euclidean distance calculation: sqrt(dx^2 + dy^2 + dz^2)
        dist_to_wp = math.sqrt(dx**2 + dy**2 + dz**2)

        # Adaptive Velocity Scaling
        dist_to_x_wall = X_GEOFENCE_LIMIT - abs(current_x)
        dist_to_y_wall = Y_GEOFENCE_LIMIT - abs(current_y)
        dist_to_z_ceiling = abs(Z_MAX_ALTITUDE - current_z)
        dist_to_z_floor = abs(Z_MIN_ALTITUDE - current_z)

        # distance to nearest wall
        dist_to_z_wall = min(dist_to_z_ceiling, dist_to_z_floor)  # vertical
        min_wall_dist = min(dist_to_x_wall, dist_to_y_wall, dist_to_z_wall)

        SLOWDOWN_ZONE = 8.0  # 8 meters from the fence, it slowly drops velocity
        if min_wall_dist < SLOWDOWN_ZONE:
            # linear speed based on wall proximity
            fence_speed = max(0.15, min_wall_dist / SLOWDOWN_ZONE)
        else:
            fence_speed = 1.0
        # adaptive speed limits
        adaptive_max_horiz = MAX_SPEED_HORIZ * fence_speed
        adaptive_max_vert = MAX_SPEED_VERT * fence_speed

        # target velocities based on adaptive limits
        target_vx = max(-adaptive_max_horiz, min(adaptive_max_horiz, KP_GAIN * dx))
        target_vy = max(-adaptive_max_horiz, min(adaptive_max_horiz, KP_GAIN * dy))
        target_vz = max(-adaptive_max_vert, min(adaptive_max_vert, KP_GAIN * dz))
        print(f"[FLYING WP{current_wp_index+1}] Pos: ({current_x:.2f}, {current_y:.2f}, {current_z:.2f})m | "
              f"Actual Speed: ({current_vx:.2f}, {current_vy:.2f}, {current_vz:.2f})m/s | "
              f"Cmd Speed: ({target_vx:.2f}, {target_vy:.2f}, {target_vz:.2f})m/s")

        # Check if target waypoint is reached (within 1.5m 3D tolerance)
        if dist_to_wp <= 1.5:
            print(f"\n[WAYPOINT {current_wp_index + 1} REACHED] At ({current_x:.2f}, {current_y:.2f}, {current_z:.2f})m")
            if current_wp_index + 1 < len(waypoints):
                current_wp_index += 1
                print(f"Advancing to Next Waypoint: {waypoints[current_wp_index]}\n")
            else:
                print("All waypoints complete! Holding position.")
                hold_x, hold_y, hold_z = current_x, current_y, current_z
                state = "HOLDING"
                continue

    elif state == "RECOVERING":
        # Vector pointing back toward safe center (0, 0, -10m)
        dx = 0.0 - current_x
        dy = 0.0 - current_y
        dz = SAFE_ALTITUDE - current_z

        # Dynamic recovery velocity vector back into safe zone
        target_vx = max(-5.0, min(5.0, KP_GAIN * dx))  # Reverse speed cap at 5 m/s
        target_vy = max(-5.0, min(5.0, KP_GAIN * dy))
        target_vz = max(-2.0, min(2.0, KP_GAIN * dz))  # Vertical recovery speed cap at 2 m/s

        # Check if drone has safely pulled back inside boundary threshold
        in_xy_safe = abs(current_x) <= SAFE_RADIUS and abs(current_y) <= SAFE_RADIUS
        in_z_safe = Z_MAX_ALTITUDE <= current_z <= Z_MIN_ALTITUDE

        current_speed = math.sqrt(current_vx**2 + current_vy**2 + current_vz**2)
        if in_xy_safe and in_z_safe and current_speed <= 0.2:
            print(f"\n*** RECOVERY COMPLETE AT ({current_x:.2f}, {current_y:.2f}, {current_z:.2f})m! ***\n")
            hold_x, hold_y, hold_z = current_x, current_y, current_z
            # Skip invalid breached waypoint and advance to next target
            if current_wp_index + 1 < len(waypoints):
                current_wp_index += 1
                print(f"--> ADVANCING TO WAYPOINT {current_wp_index + 1}: {waypoints[current_wp_index]}\n")
                state = "FLYING"
            else:
                hold_x, hold_y, hold_z = current_x, current_y, current_z
                state = "HOLDING"
            print(f"[RECOVERING] Pos: ({current_x:.2f}, {current_y:.2f}, {current_z:.2f})m | "
                  f"Cmd Speed: ({target_vx:.2f}, {target_vy:.2f}, {target_vz:.2f})m/s")

    elif state == "HOLDING":
        # Command zero velocity during holding state
        target_vx = 0.0
        target_vy = 0.0
        target_vz = 0.0
        print(f"[{state}] Target WP:{target_wp} | Pos: ({current_x:.1f}, {current_y:.1f}, {current_z:.1f})m")

    # MAVLINK COMMAND ISSUANCE
    if state == "HOLDING":
        # Switch to POSITION CONTROL bitmask to freeze quadcopter in place (NO DRIFT)
        the_connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, the_connection.target_system, the_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(mask_position),
                hold_x, hold_y, hold_z, 0, 0, 0, 0, 0, 0, 0, 0
            )
        )
    else:
        # Stream VELOCITY CONTROL bitmask to move toward waypoints or recover
        the_connection.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                10, the_connection.target_system, the_connection.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(mask_velocity),
                0, 0, 0, target_vx, target_vy, target_vz, 0, 0, 0, 0, 0
            )
        )
