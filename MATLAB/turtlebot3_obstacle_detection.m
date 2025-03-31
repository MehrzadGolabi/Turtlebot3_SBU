function turtlebot3_obstacle_detection()
% TurtleBot3 Obstacle Detection - MATLAB version
%
% Subscribes:
%   - /scan        (sensor_msgs/LaserScan)
%   - /cmd_vel_raw (geometry_msgs/Twist)
%
% Publishes:
%   - /cmd_vel     (geometry_msgs/Twist)
%
% If an obstacle is detected within 0.5 m in the ±90° front sector,
% the linear velocity is forced to zero (the robot stops).
% Otherwise, the robot uses the teleop command received from /cmd_vel_raw.

    % Print intro
    disp('TurtleBot3 Obstacle Detection - Auto Move Enabled');
    disp('----------------------------------------------');
    disp('Stop angle: ±90° around the front (index ~0)');
    disp('Stop distance: 0.5 m');
    disp('----------------------------------------------');

    %--- Create a ROS 2 node in MATLAB ---
    nodeName = "turtlebot3_obstacle_detection";
    setenv('ROS_DOMAIN_ID','30');
    node     = ros2node(nodeName);

    %--- Create subscribers ---
    % LaserScan (e.g., 360 beams if 1° resolution)
    scanSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan");

    % "Raw" velocity commands from teleop or other source
    cmdVelRawSub = ros2subscriber(node, "/cmd_vel_raw", "geometry_msgs/Twist");

    %--- Create publisher for final velocity commands ---
    cmdVelPub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");

    %--- Prepare messages and parameters ---
    % Stop distance threshold
    stopDistance = 0.2;  % meters
    % Default twist in case /cmd_vel_raw not received yet
    teleTwistMsg = ros2message("geometry_msgs/Twist");
    teleTwistMsg.linear.x  = 0.2;
    teleTwistMsg.angular.z = 0.0;

    % For sending final commands
    twistMsg = ros2message("geometry_msgs/Twist");

    %--- Use ros2rate for the main loop at 10 Hz ---
    loopRate = ros2rate(node,10);

    disp('Node is running... Press Ctrl+C or stop the script to exit.');
    scanReceived = false;

    while true
        % 1) Get the latest LaserScan
        newScan = scanSub.LatestMessage;
        if ~isempty(newScan)
            scanRanges = newScan.ranges;
            scanReceived = true;
        end

        % 2) Get the latest /cmd_vel_raw
        newCmdVelRaw = cmdVelRawSub.LatestMessage;
        if ~isempty(newCmdVelRaw)
            teleTwistMsg = newCmdVelRaw;
        end

        % 3) If we have valid scan data, check obstacle
        if scanReceived
            twistMsg = detectObstacle(scanRanges, teleTwistMsg, stopDistance);
        else
            % If no scan yet, just use whatever teleTwist we have
            twistMsg = teleTwistMsg;
        end

        % 4) Publish final velocity command
        send(cmdVelPub, twistMsg);

        % Wait to maintain 10 Hz loop
        waitfor(loopRate);
    end

end


function twistOut = detectObstacle(scanRanges, teleTwistMsg, stopDistance)
% DETECTOBSTACLE - Logic that checks if any obstacle in the ±90° front sector is within stopDistance.
% If so, override linear.x to 0.0 and keep teleTwistMsg.angular.z.
% Otherwise, pass teleTwistMsg as is.

    n = numel(scanRanges);
    if n < 2
        % No valid scan data => return teleop
        twistOut = teleTwistMsg;
        return;
    end

    % The original Python code checked the front ±90° as:
    %   left_range  = n/4
    %   right_range = 3n/4
    % Then it took ranges[0:left_range] and ranges[right_range: n]
    %
    % For a 360-beam LIDAR, that means 0..90 and 270..359 => ±90° from the front (index=0).
    leftIdx  = floor(n/4);    % ~90
    rightIdx = floor(3*n/4);  % ~270

    % Replace NaN/Inf with large number so they don't cause min() to fail
    validRanges = scanRanges;
    invalidMask = ~isfinite(validRanges);
    validRanges(invalidMask) = 999.0;

    % Minimum distance in the front ±90° sector
    frontMin = min( ...
        [ validRanges(1 : leftIdx); ...
          validRanges(rightIdx+10 : end) ] );

    % Construct Twist message to publish
    twistOut = ros2message("geometry_msgs/Twist");

    if frontMin < stopDistance
        % Obstacle: override linear.x = 0, keep teleop angular.z
        twistOut.linear.x  = 0.0;
        twistOut.angular.z = teleTwistMsg.angular.z;

        % Print a warning (throttled if desired)
        disp('Obstacle detected! Stopping forward motion...');
    else
        % No obstacle => use the teleop command
        twistOut = teleTwistMsg;
    end
end
