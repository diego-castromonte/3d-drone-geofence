% live_drone_dashboard_2D.m
% 2D Multi-Drone Telemetry & Altitude Dashboard with Event Annotations

clear; clc; close all;

% Initialize UDP Receiver Socket on Port 5005
udpRx = udpport("datagram", "LocalPort", 5005);

% AUTOMATIC CLEANUP: Clears port 5005 automatically whenever script stops or figure closes
cleanUp = onCleanup(@() clear('udpRx'));

% Prepare Main Figure
fig = figure('Name', 'Multi-Drone 2D Telemetry & Altitude Dashboard', ...
             'Color', [0.12 0.12 0.12], 'Position', [100, 100, 1000, 750]);

% --- TOP SUBPLOT: 2D Spatial View (North vs. East) ---
ax1 = subplot(2, 1, 1, 'Parent', fig);
set(ax1, 'Color', [0.18 0.18 0.18], 'XColor', 'w', 'YColor', 'w');
hold(ax1, 'on'); grid(ax1, 'on'); 

% Aspect ratio & padded limits so legends/labels don't clip
axis(ax1, 'equal');
pbaspect(ax1, [1 1 1]);
xlim(ax1, [-38 38]); ylim(ax1, [-38 38]);
xlabel(ax1, 'East (Y) [m]'); ylabel(ax1, 'North (X) [m]');
title(ax1, '2D Top-Down Flight Path & Boundary Tracking', 'Color', 'w', 'FontSize', 12);

% Draw 2D Geofence Perimeter (+/- 25m)
fenceX = [-25, 25, 25, -25, -25];
fenceY = [-25, -25, 25, 25, -25];
plot(ax1, fenceY, fenceX, 'r--', 'LineWidth', 2, 'DisplayName', 'Geofence Perimeter');

% Draw 8m Slowdown Buffer Zone Perimeter (+/- 17m)
slowX = [-17, 17, 17, -17, -17];
slowY = [-17, -17, 17, 17, -17];
plot(ax1, slowY, slowX, 'y:', 'LineWidth', 1.2, 'DisplayName', 'Slowdown Zone (8m Buffer)');

% --- BOTTOM SUBPLOT: Altitude vs. Time Profile ---
ax2 = subplot(2, 1, 2, 'Parent', fig);
set(ax2, 'Color', [0.18 0.18 0.18], 'XColor', 'w', 'YColor', 'w');
hold(ax2, 'on'); grid(ax2, 'on');
xlim(ax2, [0 60]); ylim(ax2, [-25 2]);
xlabel(ax2, 'Elapsed Time [s]'); ylabel(ax2, 'Altitude (Z) [m]');
title(ax2, 'Real-Time Vehicle Altitude Profile', 'Color', 'w', 'FontSize', 12);

% Draw Altitude Boundaries (Z Ceiling -20m, Floor -3m)
yline(ax2, -20, 'r--', 'Ceiling (-20m)', 'Color', [1 0.4 0.4], 'LineWidth', 1.5, 'DisplayName', 'Altitude Ceiling');
yline(ax2, -3, 'w--', 'Floor (-3m)', 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5, 'DisplayName', 'Altitude Floor');

% Color Palette for 3 Drones
colors = ['#00FFFF'; '#FF00FF'; '#FFFF00'];  % Cyan, Magenta, Yellow

% Handles for Data History
hTrail2D = cell(1,3); hHead2D = cell(1,3);
hAltLine = cell(1,3);
histX = cell(1,3); histY = cell(1,3); histZ = cell(1,3); histT = cell(1,3);

% Track breach state per drone to debounce markers (1 icon per event)
was_breached = [false, false, false];

for i = 1:3
    histX{i} = []; histY{i} = []; histZ{i} = []; histT{i} = [];
    
    % 2D Map Handles
    hTrail2D{i} = plot(ax1, NaN, NaN, 'Color', colors(i,:), 'LineWidth', 1.8, 'DisplayName', sprintf('Drone %d Path', i));
    hHead2D{i}  = plot(ax1, NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'w', 'HandleVisibility', 'off');
    
    % Altitude Profile Handles
    hAltLine{i} = plot(ax2, NaN, NaN, 'Color', colors(i,:), 'LineWidth', 1.8, 'DisplayName', sprintf('Drone %d Alt', i));
end

% Place legends cleanly inside without squishing plot dimensions
legend(ax1, 'TextColor', 'w', 'Location', 'northeast');
legend(ax2, 'TextColor', 'w', 'Location', 'northeast');

% Position Timer Box cleanly in the top-left quadrant inside the padded plot
hTimeText = text(ax1, -35, 33, 'Time: 0.0s', 'Color', 'w', 'FontSize', 10, ...
                 'FontWeight', 'bold', 'BackgroundColor', [0.1 0.1 0.1], 'EdgeColor', 'w');

startTime = [];
disp('MATLAB 2D Dashboard Listening on UDP Port 5005...');

% --- REAL-TIME PROCESSING LOOP ---
while ishandle(fig)
    if udpRx.NumDatagramsAvailable > 0
        dataGram = read(udpRx, udpRx.NumDatagramsAvailable, "string");
        latestPacket = dataGram(end).Data;
        
        try
            packet = jsondecode(latestPacket);
            
            if isempty(startTime)
                startTime = tic;
            end
            currentTime = toc(startTime);
            
            % Update Mission Timer Display
            set(hTimeText, 'String', sprintf('Time: %.1fs', currentTime));
            
            % Update Time Window Axis dynamically
            if currentTime > 60
                xlim(ax2, [currentTime - 60, currentTime + 5]);
            end
            
            % Process Telemetry for Each Drone
            for i = 1:3
                fieldName = sprintf('d%d', i);
                if isfield(packet, fieldName)
                    pos = packet.(fieldName);
                    cx = pos(1); cy = pos(2); cz = pos(3);
                    
                    histX{i} = [histX{i}, cx];
                    histY{i} = [histY{i}, cy];
                    histZ{i} = [histZ{i}, cz];
                    histT{i} = [histT{i}, currentTime];
                    
                    % 1. Update 2D Trajectory Map (Y is East, X is North)
                    set(hTrail2D{i}, 'XData', histY{i}, 'YData', histX{i});
                    set(hHead2D{i}, 'XData', cy, 'YData', cx);
                    
                    % 2. Update Altitude vs. Time Profile
                    set(hAltLine{i}, 'XData', histT{i}, 'YData', histZ{i});
                    
                    % --- DEBOUNCED GEOFENCE BREACH ANNOTATIONS ---
                    is_breached = (abs(cx) >= 25 || abs(cy) >= 25 || cz <= -20 || cz >= -3);
                    
                    if is_breached && ~was_breached(i)
                        plot(ax1, cy, cx, 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
                        plot(ax2, currentTime, cz, 'rv', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
                    end
                    
                    was_breached(i) = is_breached;
                end
            end
            
            % Inter-Drone Proximity Check Annotation
            for i = 1:3
                for j = i+1:3
                    if ~isempty(histX{i}) && ~isempty(histX{j})
                        p1 = [histX{i}(end), histY{i}(end), histZ{i}(end)];
                        p2 = [histX{j}(end), histY{j}(end), histZ{j}(end)];
                        dist = norm(p1 - p2);
                        
                        if dist < 4.0
                            plot(ax1, [p1(2), p2(2)], [p1(1), p2(1)], 'r:', 'LineWidth', 2, 'HandleVisibility', 'off');
                        end
                    end
                end
            end
            
            drawnow limitrate;
            
        catch
            % Skip malformed UDP packets
        end
    end
    pause(0.01);
end