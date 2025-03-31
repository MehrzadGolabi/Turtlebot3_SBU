% Clear previous ROS connections
clear; clc;

% Initialize a local ROS 2 node in MATLAB (adjust DomainID and NodeName if necessary)
nodeName = "turtlebot3_position_control_node";
setenv('ROS_DOMAIN_ID','30');
node     = ros2node(nodeName); 

% Create a subscriber for /odom (nav_msgs/Odometry)
odomSub = ros2subscriber(node, "/odom", "nav_msgs/Odometry");

% Create a publisher for /cmd_vel (geometry_msgs/Twist)
cmdVelPub = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");

% Create a message object for publishing velocity
cmdMsg = ros2message("geometry_msgs/Twist");

disp('MATLAB node created. Ready to receive and send messages...');


% Ask user for a goal in absolute coordinates:
goal_x = str2double(input('Enter goal x (absolute): ', 's'));
goal_y = str2double(input('Enter goal y (absolute): ', 's'));
goal_heading_deg = str2double(input('Enter goal heading (absolute, deg): ', 's'));

% Convert degrees to radians and keep within (-pi, pi)
goal_heading = deg2rad(goal_heading_deg);
if goal_heading > pi
    goal_heading = goal_heading - 2*pi;
elseif goal_heading < -pi
    goal_heading = goal_heading + 2*pi;
end

disp(['New goal: x=' num2str(goal_x) ', y=' num2str(goal_y) ...
      ', heading=' num2str(rad2deg(goal_heading)) ' deg']);

% Control gains / parameters
linearSpeed  = 1;
angularSpeedLimit = 1.5;
distanceThreshold = 0.05;  % [m] stop if within 5 cm
headingThreshold  = deg2rad(1.0);  % ~1 degree

% Control loop
rate = ros2rate(node,20);  % 20 Hz loop
done = false;

while ~done
    % Get the latest odometry
    odomMsg = odomSub.LatestMessage;
    if isempty(odomMsg)
        % Wait until first message arrives
        waitfor(rate);
        continue;
    end

    % Current position
    pos = odomMsg.pose.pose.position;
    robotX = pos.x;
    robotY = pos.y;

    % Current orientation (as roll, pitch, yaw)
    orientation = odomMsg.pose.pose.orientation;
    [~, ~, robotYaw] = quatToRPY(orientation);

    % Position error
    dx = goal_x - robotX;
    dy = goal_y - robotY;
    distance = sqrt(dx^2 + dy^2);

    if distance > distanceThreshold
        % We still need to move linearly toward the goal
        goalDirection = atan2(dy, dx);
        pathAngle     = goalDirection - robotYaw;

        % Normalize pathAngle to (-pi, pi)
        if pathAngle > pi
            pathAngle = pathAngle - 2*pi;
        elseif pathAngle < -pi
            pathAngle = pathAngle + 2*pi;
        end

        % Assign angular velocity
        % Limit the turn speed
        cmdMsg.angular.z = max(min(pathAngle, angularSpeedLimit), -angularSpeedLimit);

        % Linear speed scaled by distance (but limit it to e.g. 0.1)
        cmdMsg.linear.x = min(linearSpeed * distance, 0.1);

        disp(['Moving to x=' num2str(goal_x) ', y=' num2str(goal_y) ...
              ' (current=' num2str(robotX) ',' num2str(robotY) ')']);

        send(cmdVelPub, cmdMsg);

    else
        % Close enough in position, now rotate to final heading
        headingError = goal_heading - robotYaw;

        % Normalize headingError to (-pi, pi)
        if headingError > pi
            headingError = headingError - 2*pi;
        elseif headingError < -pi
            headingError = headingError + 2*pi;
        end

        % If heading not yet correct, rotate in place
        if abs(headingError) > headingThreshold
            turnSpeed = max(min(abs(headingError) * 1.0, 1.0), 0.1);
            cmdMsg.linear.x  = 0.0;
            cmdMsg.angular.z = sign(headingError)*turnSpeed;

            disp(['Rotating to heading=' num2str(rad2deg(goal_heading)) ...
                  ' (current=' num2str(rad2deg(robotYaw)) ' deg)']);

            send(cmdVelPub, cmdMsg);
        else
            % We have arrived
            cmdMsg.linear.x  = 0.0;
            cmdMsg.angular.z = 0.0;
            send(cmdVelPub, cmdMsg);

            disp(['Goal reached: x=' num2str(goal_x) ...
                  ', y=' num2str(goal_y) ...
                  ', heading=' num2str(rad2deg(goal_heading)) ' deg']);

            % Optionally prompt for next goal or end
            disp('Waiting for next goal...');
            goal_x = str2double(input('Enter next goal x: ', 's'));
            goal_y = str2double(input('Enter next goal y: ', 's'));
            goal_heading_deg = str2double(input('Enter next heading (deg): ', 's'));

            goal_heading = deg2rad(goal_heading_deg);
            if goal_heading > pi
                goal_heading = goal_heading - 2*pi;
            elseif goal_heading < -pi
                goal_heading = goal_heading + 2*pi;
            end

            disp(['New goal: x=' num2str(goal_x) ', y=' num2str(goal_y) ...
                  ', heading=' num2str(rad2deg(goal_heading)) ' deg']);
        end
    end

    waitfor(rate);  % maintain 20 Hz loop
end

function [roll, pitch, yaw] = quatToRPY(q)
    % q is a struct or array with fields (w, x, y, z)
    % The same standard formula as used in the Python code:
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll      = atan2(sinr_cosp, cosr_cosp);

    sinp      = 2.0 * (q.w * q.y - q.z * q.x);
    pitch     = asin(sinp);

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw       = atan2(siny_cosp, cosy_cosp);
end
