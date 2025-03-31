% live_mapping.m
function live_mapping()
% LIVE_MAPPING - Continuously builds an expanding 2D map from /scan & /odom.
%                Draws the current robot position in the map.
%
% 1) Subscribes to /scan (LaserScan) & /odom (Odometry).
% 2) Converts each scan to global (odom) frame using the robot pose.
% 3) Accumulates points in a persistent "mapPoints" array.
% 4) Displays the map and robot in real time.

    %% 1. Set up ROS 2
    % Make sure your ROS_DOMAIN_ID is set appropriately:
    %   setenv('ROS_DOMAIN_ID','30');
    % or do so externally before starting MATLAB.

    nodeName = "live_mapping_node";
    node     = ros2node(nodeName);

    % 2. Create subscribers
    scanSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan");
    odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");

    % 3. Prepare figure for real-time display
    figure('Name', 'TurtleBot3 Live Mapping');
    hold on; grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]');
    title('Live 2D Map (Odom Frame)');

    % We'll store all points in a Nx2 array
    mapPoints = [];  % [x, y] in odom frame

    % 4. Set up a loop rate of 5 Hz (for example)
    r = ros2rate(node,5);

    disp('Starting live mapping...');
    disp('Press CTRL+C or stop the script to end.');

    while true
        % Attempt to get the latest data
        scanMsg = scanSub.LatestMessage;
        odomMsg = odomSub.LatestMessage;

        if ~isempty(scanMsg) && ~isempty(odomMsg)
            % 5. Get the robot pose from /odom
            [rx, ry, rtheta] = getPoseFromOdom(odomMsg);

            % 6. Convert scan to (x,y) in robot frame
            [scanX, scanY] = laserToCartesian(scanMsg);

            % 7. Transform (scanX, scanY) to odom frame
            [globalX, globalY] = localToGlobal(scanX, scanY, rx, ry, rtheta);

            % 8. Accumulate points
            % Optionally, you can filter out Inf or NaN
            valid = isfinite(globalX) & isfinite(globalY);
            newPoints = [globalX(valid), globalY(valid)];
            mapPoints = [mapPoints; newPoints]; %#ok<AGROW>

            % (Optional) If you want to remove old points or limit memory,
            % you can implement a more complex approach here.

            % 9. Refresh the plot
            % Clear axes or incrementally update
            cla;
            % Plot all map points
            plot(mapPoints(:,1), mapPoints(:,2), '.b', 'MarkerSize', 1); hold on;
            % Draw robot
            drawRobot(rx, ry, rtheta);
            drawnow;
        end

        waitfor(r); % maintain loop at ~5 Hz
    end
end


%% ---- Helper Functions ---------------------------------------------------

function [x, y, theta] = getPoseFromOdom(odomMsg)
% Extract 2D pose (x, y, yaw) from nav_msgs/Odometry
    pose = odomMsg.pose.pose;
    x = pose.position.x;
    y = pose.position.y;

    % Convert quaternion to euler
    q = pose.orientation;
    yaw = quat2eul([q.w q.x q.y q.z]); % see quaternion note below
    theta = yaw;  % Just take the yaw
end


function [xData, yData] = laserToCartesian(scanMsg)
% Convert LaserScan to Cartesian coordinates in the local (base_link) frame
%
% Ranges are an array of distances
% The angles typically start at angle_min and increment by angle_increment
%
% This function returns arrays xData, yData the same size as scanMsg.ranges

    ranges = scanMsg.ranges;

    angleMin  = scanMsg.angle_min;      % start angle
    angleInc  = scanMsg.angle_increment;
    n = numel(ranges);

    % Construct an array of angles
    angles = angleMin + (0:(n-1))' * angleInc;

    % Convert to cartesian: x = r*cos(theta), y = r*sin(theta)
    % We ignore infinite or NaN ranges (set them to NaN)
    rValid = ranges;
    rValid(~isfinite(rValid)) = NaN;
    xData = rValid .* cos(angles);
    yData = rValid .* sin(angles);
end


function [gx, gy] = localToGlobal(xData, yData, rx, ry, rtheta)
% Transform points (xData, yData) in the robot's local frame
% to the odom (global) frame, given the robot's pose (rx, ry, rtheta).

    % 2D rotation, then translation:
    cosT = cos(rtheta);
    sinT = sin(rtheta);

    gx = rx + cosT.*xData - sinT.*yData;
    gy = ry + sinT.*xData + cosT.*yData;
end


function drawRobot(rx, ry, rtheta)
% Draw a simple triangular representation of the robot
% at pose (rx, ry, rtheta).

    % Robot shape (triangle) in local coords
    robot_radius = 0.1; % approximate half the TB3 footprint
    L = robot_radius*2;
    % A simple arrow shape
    pts = [0.5*L,   0;   -0.5*L,  0.3*L;   -0.5*L, -0.3*L;  0.5*L, 0];
    % Rotate+translate
    R = [cos(rtheta), -sin(rtheta); sin(rtheta), cos(rtheta)];
    ptsGlobal = (R * pts')';
    ptsGlobal(:,1) = ptsGlobal(:,1) + rx;
    ptsGlobal(:,2) = ptsGlobal(:,2) + ry;

    % Plot
    patch('XData', ptsGlobal(:,1), 'YData', ptsGlobal(:,2), ...
          'FaceColor', 'r', 'EdgeColor', 'k');
end


function yaw = quat2eul(quat)
% QUAT2EUL - minimal custom function to extract yaw from [w x y z]
% Alternative: you can use the built-in "quaternion" or "eulerd" in newer MATLAB releases.
%
%   quat is [w x y z]
%   returns yaw in the range [-pi, pi].
%
    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);

    % Roll (phi) not used here
    % Pitch (theta) not used here
    % Yaw (psi):
    siny_cosp = 2.0*(w*z + x*y);
    cosy_cosp = 1.0 - 2.0*(y*y + z*z);
    yaw = atan2(siny_cosp, cosy_cosp);
end
