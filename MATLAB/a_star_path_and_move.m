function a_star_path_and_move()
%--------------------------------------------------------------------------
% A_STAR_PATH_AND_MOVE_DEBUG_05:
%  Adapts the previous code to use a map resolution of 0.05 m/pixel (5 cm).
%  1) Loads myMap.pgm as a binaryOccupancyMap with resolution=0.05.
%  2) Inflates obstacles for a robot radius ~0.15 m.
%  3) Prompts user for three clicks: start pose, orientation reference, goal.
%  4) Plans a path with A* using world2grid logic (unchanged).
%  5) Connects to ROS 2 (/odom, /cmd_vel), moves the robot, and prints debug info.
%
% NOTE:
%  - The scale is 5 cm per pixel => if your robot is e.g. 30 cm diameter, use
%    robotRadius ~ 0.15. Tune thresholds/speeds accordingly.
%--------------------------------------------------------------------------
node = ros2node("/a_star_debug_mover_05");
odomSub   = ros2subscriber(node,"/odom","nav_msgs/Odometry");
cmdVelPub = ros2publisher(node,"/cmd_vel","geometry_msgs/Twist");

%% 1) Read the map image
load myMapGrid.mat
finalOrigin;
mapImage = imread('myMap.pgm');  % or specify full path

% Threshold (<128 => occupied)
binaryMap = mapImage < 128;

%% 2) Create binaryOccupancyMap with resolution=0.05
resolution = 0.05;  % 5 cm per pixel
map = binaryOccupancyMap(binaryMap,1/resolution);

%% 3) Inflate by robot radius
% For a ~30 cm diameter robot, radius ~0.15
robotRadius = 0.15;
inflate(map, robotRadius);

%% Prompt for 3 clicks: start, orientation, goal
figure('Name','Select Robot Pose, Orientation, Goal');
show(map);
hold on;
title(['کلیک اول: مکان ربات | کلیک دوم: زاویه اولیه | کلیک سوم: هدف']);

[xClicks, yClicks] = ginput(1);

manualPose = [0, 0, 0];
% صبر تا دریافت اولین پیام اُدومتری
fprintf('در حال دریافت داده‌های odom...\n');
pause(2);  % زمان کوتاه برای دریافت پیام
currentPose = getCurrentPoseWithFallback(odomSub, manualPose);

robotStartPos    = [currentPose(1) + abs(finalOrigin(1)), currentPose(2) + abs(finalOrigin(2))]
orientationPoint = [0, 0];
goalPos          = [xClicks(1), yClicks(1)];

% Compute initial yaw from first to second click
dx = orientationPoint(1) - robotStartPos(1);
dy = orientationPoint(2) - robotStartPos(2);
userInitialYaw = atan2(dy, dx);

plot(robotStartPos(1),   robotStartPos(2),   'go','MarkerFaceColor','g');
plot(orientationPoint(1),orientationPoint(2),'gx','LineWidth',2);
plot(goalPos(1),         goalPos(2),         'bo','MarkerFaceColor','b');
hold off;

% Check occupancy
if checkOccupancy(map, robotStartPos)
    error('نقطه شروع ربات در ناحیه اشغال شده است!');
end
if checkOccupancy(map, goalPos)
    error('نقطه هدف در ناحیه اشغال شده است!');
end

%% Convert to grid coords (unchanged)
startGrid = world2grid(map, robotStartPos);
goalGrid  = world2grid(map, goalPos);

%% A* planner
planner = plannerAStarGrid(map);
[path, ~] = plan(planner, startGrid, goalGrid);

% Convert path => world
worldPath = grid2world(map, path);

%% Display path
figure('Name','A* Path (Resolution=0.05)');
show(map); 
hold on;
plot(worldPath(:,1), worldPath(:,2), 'r-', 'LineWidth',2);
plot(robotStartPos(1), robotStartPos(2), 'gs','MarkerSize',8,'MarkerFaceColor','g');
plot(goalPos(1),       goalPos(2),       'bs','MarkerSize',8,'MarkerFaceColor','b');
title('مسیر برنامه‌ریزی شده با A* (مقیاس ۵سانتی در پیکسل)');
legend('Path','Start','Goal');
hold off;
worldPath(:,1) = worldPath(:,1) -abs(finalOrigin(1));
worldPath(:,2) = worldPath(:,2) -abs(finalOrigin(2));
goalPos(1) = goalPos(1) - abs(finalOrigin(1));
goalPos(2) = goalPos(2) - abs(finalOrigin(2));
%--------------------------------------------------------------------------
%  Control with ROS 2, printing debug info
%--------------------------------------------------------------------------

% Adjust control params to your scale
linSpeedMax   = 8.2;           % 0.2 m/s (faster if you want)
angSpeedMax   = 1.5;           % rad/s
distThreshold = 0.1;           % 10 cm from a waypoint
angThreshold  = deg2rad(10);   % 10 degrees

% Fallback pose if odom not available
%manualPose = [goalPos(1),       goalPos(2), userInitialYaw];

for i = 1:size(worldPath,1)
    targetXY = worldPath(i,:);
    fprintf('\n=== رفتن به نقطه %d: (%.2f, %.2f) ===\n', i, targetXY(1), targetXY(2));

    while true
        currentPose = getCurrentPoseWithFallback(odomSub, manualPose);

        % Print current pose
        fprintf('[DEBUG] Pose: x=%.2f, y=%.2f, yaw=%.2f deg\n',...
            currentPose(1), currentPose(2), rad2deg(currentPose(3)));

        dx   = targetXY(1) - currentPose(1);
        dy   = targetXY(2) - currentPose(2);
        dist = hypot(dx, dy);
        if dist < distThreshold
            fprintf('[DEBUG] به اندازه کافی نزدیک هستیم (Dist=%.2f < %.2f)\n', dist, distThreshold);
            break;
        end

        desiredHeading = atan2(dy, dx);
        headingError   = wrapToPi(desiredHeading - currentPose(3));

        fprintf('[DEBUG] Target=(%.2f,%.2f), Dist=%.2f, HeadingErr=%.2f deg\n',...
            targetXY(1), targetXY(2), dist, rad2deg(headingError));

        if abs(headingError) > angThreshold
            linSpeed = 0.0;
            angSpeed = sign(headingError)*min(abs(headingError), angSpeedMax);
        else
            linSpeed = min(linSpeedMax, 0.5*dist);
            angSpeed = sign(headingError)*min(abs(headingError), angSpeedMax);
        end

        fprintf('[DEBUG] linSpeed=%.2f, angSpeed=%.2f\n', linSpeed, angSpeed);

        % Publish /cmd_vel
        cmdMsg = ros2message("geometry_msgs/Twist");
        cmdMsg.linear.x  = linSpeed;
        cmdMsg.angular.z = angSpeed;
        send(cmdVelPub, cmdMsg);

        pause(0.2);
    end
end

% Stop
stopRobot(cmdVelPub);
disp('تمام نقاط مسیر طی شد. برنامه پایان یافت.');
end


%% --------------------- getCurrentPoseWithFallback --------------------
function pose = getCurrentPoseWithFallback(odomSub, fallbackPose)
odomMsg = odomSub.LatestMessage;
if isempty(odomMsg)
    pose = fallbackPose;
    return;
end

p   = odomMsg.pose.pose.position;
q   = odomMsg.pose.pose.orientation;
x   = double(p.x);
y   = double(p.y);
qw  = q.w; qx = q.x; qy = q.y; qz = q.z;

siny_cosp = 2*(qw*qz + qx*qy);
cosy_cosp = 1 - 2*(qy^2 + qz^2);
yaw       = atan2(siny_cosp, cosy_cosp);

pose = [x, y, yaw];
end


%% --------------------- stopRobot -------------------------------------
function stopRobot(pub)
cmdMsg = ros2message("geometry_msgs/Twist");
cmdMsg.linear.x  = 0.0;
cmdMsg.angular.z = 0.0;
send(pub, cmdMsg);
end
