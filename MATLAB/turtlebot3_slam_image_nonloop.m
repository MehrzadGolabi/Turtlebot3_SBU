function turtlebot3_slam()
% TURTLEBOT3_SLAM - Real-time 2D SLAM using Lidar scans from a TurtleBot3
% (ROS 2). Incorporates loop closure detection for a globally consistent map.
%
% Requirements:
% - Navigation Toolbox (for lidarSLAM)
% - ROS Toolbox with ROS 2 support
% - A TurtleBot3 publishing /scan (and optionally /odom)
%
% Steps:
% 1) Subscribes to /scan and /odom
% 2) Converts each incoming LaserScan into a lidarScan object
% 3) Adds scans to the lidarSLAM object, which performs scan matching, loop closure
% 4) Plots the evolving map after each added scan
%
% You may want to adjust parameters (e.g., 'MaxNumScans') for long runs.

    % 1. Create a ROS 2 node in MATLAB
    nodeName = "turtlebot3_slam_node";
    setenv('ROS_DOMAIN_ID','30');
    node     = ros2node(nodeName);

    % 2. Create subscribers
    % LaserScan: typically 360 or 720 beams around the robot
    scanSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan");
    % Odometry: used as an initial guess to speed up scan matching (optional)
    % If you don't have /odom or don't trust it, you can skip or remove this
    odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");

    % 3. Configure the lidarSLAM object
    % Adjust resolution or loop closure thresholds as needed
    mapResolution = 20;    % cells per meter (grid resolution)
    maxScans      = 800;   % max number of scans to keep in the SLAM pose graph
    slamObj = lidarSLAM(mapResolution, maxScans);

    % Tune these parameters based on your environment
    slamObj.LoopClosureThreshold    = 210;  % lower => more loop closures
    slamObj.LoopClosureSearchRadius = 8;    % meters search radius for loop closures

    disp('Initialized lidarSLAM object:');
    disp(slamObj);

    % For real-time display, create a figure
    figure('Name','TurtleBot3 SLAM');
    ax = axes(); %#ok<LAXES> 
    title(ax, '2D SLAM - Map and Trajectory');
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)');

    % We'll store the current estimated pose for the next scan
    % Start at [0, 0, 0] (x,y,theta)
    currentPose = [0 0 0];

    % If we have an odometry subscriber, we can use it to update the pose guess
    useOdom = true;  % set false if you want pure scan matching

    % 4. ros2rate for main loop
    rateCtrl = ros2rate(node,20); 

    disp('Starting 2D SLAM...');
    disp('Press Ctrl+C or stop the script to end.');

    while true
        % a) Retrieve the latest LaserScan
        scanMsg = scanSub.LatestMessage;
        if isempty(scanMsg)
            % No scan yet, skip
            waitfor(rateCtrl);
            continue;
        end

        % b) Convert LaserScan => lidarScan object
        ls = rosMsgToLidarScan(scanMsg);

        if useOdom
            % c) Get odometry as an initial guess
            odomMsg = odomSub.LatestMessage;
            if ~isempty(odomMsg)
                currentPose = getPoseFromOdom(odomMsg, currentPose);
            end
            % Note: We pass currentPose to addScan for better matching
            [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamObj, ls, currentPose);
        else
            % Pure scan matching with no initial guess
            [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamObj, ls);
        end

        % d) If the scan was processed (sometimes it might be rejected)
        if isScanAccepted
             % Get all current poses from the SLAM object
             [~, poses] = scansAndPoses(slamObj);
             % The last row is the new pose for the most recent scan
             currentPose = poses(end,:);

            % e) Optionally, visualize the map every time or every few scans
            % Rebuild the occupancy map from the updated pose graph
            [scans, poses] = scansAndPoses(slamObj);
            show(slamObj, 'Parent', ax);  % or: show(slamObj, 'Poses','off','Parent',ax);
            % If you prefer a faster partial update, see doc for "lidarSLAM" optimization

            drawnow limitrate
            % Print basic loop closure info
            if loopClosureInfo.LoopClosureDetected
                disp('Loop closure detected. Refining pose graph...');
            end
        end

        waitfor(rateCtrl);
    end
end


%% ----- HELPER FUNCTIONS -------------------------------------------------

function [pose] = getPoseFromOdom(odomMsg, oldPose)
% GETPOSEFROMODOM - Extract a 2D pose [x y theta] from nav_msgs/Odometry.
% If you have a better way to track the incremental motion, do so. 
% The approach here just reads odometry's (x,y,yaw).
%
% oldPose is the previous guess, which you can use if you want to fuse 
% incremental changes. For simplicity, we just read absolute (x,y,yaw).

    p = odomMsg.pose.pose.position;
    q = odomMsg.pose.pose.orientation;

    % Convert quaternion to yaw
    orientation = [q.w q.x q.y q.z];
    yaw = quat2yaw(orientation);

    % 2D pose
    pose = [p.x, p.y, yaw];

    % If you prefer incremental:
    %   dx, dy, dtheta = difference from oldPose 
    %   pose = oldPose + [dx, dy, dtheta]
    % For demonstration, we just take absolute from Odom.
end


function yaw = quat2yaw(quat)
% QUAT2YAW - Convert [w x y z] quaternion to yaw in [-pi, pi]
    w = quat(1); x = quat(2); y = quat(3); z = quat(4);
    siny_cosp = 2*(w*z + x*y);
    cosy_cosp = 1 - 2*(y^2 + z^2);
    yaw = atan2(siny_cosp, cosy_cosp);
end


function scanObj = rosMsgToLidarScan(scanMsg)
% ROSMSGTOLIDARSCAN - Convert sensor_msgs/LaserScan to a lidarScan object 
% in MATLAB Navigation Toolbox.

    % Extract angles
    angleMin = scanMsg.angle_min;   % start
    angleInc = scanMsg.angle_increment;
    nRanges  = numel(scanMsg.ranges);
    angles = angleMin + (0:(nRanges-1))' * angleInc;

    % Convert to double row vectors
    ranges = double(scanMsg.ranges(:)');
    angles = double(angles(:)');

    % Create the lidarScan object
    scanObj = lidarScan(ranges, angles);
end
