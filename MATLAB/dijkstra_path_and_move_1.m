function dijkstra_path_and_move_1()
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

%% 1. خواندن فایل نقشه و ایجاد شیء نقشه باینری
mapImage = imread('expandedMap.pgm');   % خواندن فایل PGM
[rows, cols] = size(mapImage);
centerX = cols / 2;              
centerY = rows / 2;
binaryMap = mapImage < 128;                 % تبدیل به نقشه باینری (پیکسل‌های کمتر از 128 => اشغال)
resolution = 0.05;                          % افزایش دقت نقشه (هر سلول برابر با 0.02 متر)
map = binaryOccupancyMap(binaryMap, 1/resolution);
robotRadius = 0.15;                         % شعاع ربات به متر
inflate(map, robotRadius);                  % گسترش موانع با شعاع ربات
centerX = centerX * resolution;
centerY = centerY * resolution;
%% Prompt for 3 clicks: start, orientation, goal
figure('Name','Select Robot Pose, Orientation, Goal');
show(map);
hold on;
title(['مقصد ربات را مشخص کنید']);

[xClicks, yClicks] = ginput(1);

manualPose = [0, 0, 0];
% صبر تا دریافت اولین پیام اُدومتری
fprintf('در حال دریافت داده‌های odom...\n');
pause(2);  % زمان کوتاه برای دریافت پیام
currentPose = getCurrentPoseWithFallback(odomSub, manualPose);

robotStartPos    = [currentPose(1) + centerX, currentPose(2) + centerY]
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

% استخراج ماتریس اشغال‌پذیری از نقشه
occMat = occupancyMatrix(map);
% تبدیل به نقشه باینری: فرض کنید سلول‌هایی با مقدار > 0.5 اشغال هستند
binaryOccMat = occMat > 0.5;

[mapRows, mapCols] = size(binaryOccMat);
nodeIndex = zeros(mapRows, mapCols);
nodeCounter = 0;
for r = 1:mapRows
    for c = 1:mapCols
        if binaryOccMat(r, c) == 0   % سلول آزاد
            nodeCounter = nodeCounter + 1;
            nodeIndex(r, c) = nodeCounter;
        end
    end
end

% ایجاد یال‌ها برای سلول‌های آزاد با استفاده از همسایگی ۸-متصل
edges = [];
weights = [];
% تعریف ماتریس جابجایی جهت‌های ۸‌گانه
neighborOffsets = [ -1, -1;
                    -1,  0;
                    -1,  1;
                     0, -1;
                     0,  1;
                     1, -1;
                     1,  0;
                     1,  1];

for r = 1:mapRows
    for c = 1:mapCols
        if binaryOccMat(r, c) == 0
            currentNode = nodeIndex(r, c);
            for k = 1:size(neighborOffsets, 1)
                nr = r + neighborOffsets(k, 1);
                nc = c + neighborOffsets(k, 2);
                if nr >= 1 && nr <= mapRows && nc >= 1 && nc <= mapCols && binaryOccMat(nr, nc) == 0
                    neighborNode = nodeIndex(nr, nc);
                    % محاسبه وزن: اگر در جهات قطری (جمع قدر مطلق تغییرات برابر 2) باشد، √2، در غیر این صورت 1
                    if abs(neighborOffsets(k,1)) + abs(neighborOffsets(k,2)) == 2
                        edgeWeight = sqrt(2);
                    else
                        edgeWeight = 1;
                    end
                    edges = [edges; currentNode, neighborNode];
                    weights = [weights; edgeWeight];
                end
            end
        end
    end
end

% ایجاد گراف
G = graph(edges(:,1), edges(:,2), weights);


%% 6. تبدیل نقاط شروع و هدف به اندیس گره در گراف
% توجه: در ماتریس، سطر برابر با y و ستون برابر با x است.
startNode = nodeIndex(startGrid(2), startGrid(1));
goalNode  = nodeIndex(goalGrid(2), goalGrid(1));

%% 7. محاسبه مسیر بهینه با الگوریتم دایکسترا
[spNodes, spCost] = shortestpath(G, startNode, goalNode);

% تبدیل اندیس‌های گره به مختصات گرید
plannedPathGrid = zeros(length(spNodes), 2);
for i = 1:length(spNodes)
    [r, c] = find(nodeIndex == spNodes(i), 1);
    plannedPathGrid(i,:) = [c, r];  % [x, y] به عنوان مختصات گرید
end

%% 8. تبدیل مسیر به مختصات دنیای واقعی
worldPath = grid2world(map, plannedPathGrid);

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
worldPath(:,1) = worldPath(:,1) -abs(centerX);
worldPath(:,2) = worldPath(:,2) -abs(centerY);
goalPos(1) = goalPos(1) - abs(centerX);
goalPos(2) = goalPos(2) - abs(centerY);
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
