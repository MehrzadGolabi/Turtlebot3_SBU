function integrated_slam_path_planning()
    %INTEGRATED_SLAM_PATH_PLANNING
    % Combines:
    %   1) Dynamic occupancy map building from LIDAR (/scan) + odometry (/odom).
    %   2) Global path planning (A*) on the current map.
    %   3) Local path tracking with simple obstacle check & local replan.
    %   4) Obstacle detection to override velocities if an obstacle is too close.
    %
    % Subscribes:  /scan, /odom
    % Publishes:   /cmd_vel
    %
    % This is a conceptual approach. Real systems often use more sophisticated
    % local planners (like DWA, VFH, TEB) and more robust map-based or SLAM-based
    % localization. But this demonstrates the general idea of a 2-level planner.
    
        %---------------------%
        % 1. ROS 2 Node Setup
        %---------------------%
        setenv('ROS_DOMAIN_ID','30');
        node = ros2node("/integrated_slam_path_planning");
        scanSub   = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
        odomSub   = ros2subscriber(node,"/odom","nav_msgs/Odometry");
        cmdVelPub = ros2publisher(node,"/cmd_vel","geometry_msgs/Twist");
    
        cmdMsg = ros2message("geometry_msgs/Twist");
    
        %---------------------%
        % 2. OccupancyMap Init
        %---------------------%
        mapSize       = 20;  % meters
        mapResolution = 20;  % cells/m => 0.05 m/cell
        occMap = occupancyMap(mapSize, mapSize, mapResolution);
        occMap.LocalOriginInWorld = [-10, -10];  % shift coverage to [-10..10, -10..10]
    
        maxLidarRange = 3.5;  % typical TB3
        obstacleStopDistance = 0.4;  % if obstacle within 0.4 m => stop
    
        % Visualization
        figure('Name','SLAM + Global/Local Planning');
        ax = axes(); hold on; grid on; axis equal;
        xlabel('X (m)'); ylabel('Y (m)');
        title('Dynamic Occupancy Map + Robot + Path');
    
        %---------------------%
        % 3. Prompt for Final Goal
        %---------------------%
        disp('Enter a final goal for global path planning: ');
        [finalGoalX, finalGoalY] = getUser2DGoal();
    
        %---------------------%
        % 4. Main Loop Config
        %---------------------%
        rateCtrl = ros2rate(node,5);  % ~5 Hz
        currentPose = [0,0,0];  % [x, y, theta]
        globalPath = [];        % Nx2 array of waypoints
        currentWaypointIndex = 1;
    
        % For local motion
        linearSpeedLimit  = 0.15;  
        angularSpeedLimit = 1.5; 
        waypointThreshold = 0.15;  % If within 15 cm of the next waypoint => move on
    
        try
            while true
                % 4a) Update Map
                scanMsg = scanSub.LatestMessage;
                if ~isempty(scanMsg)
                    currentPose = maybeUpdatePose(odomSub, currentPose);
                    updateMap(occMap, scanMsg, currentPose, maxLidarRange);
                end
    
                % 4b) Obstacle detection (front sector)
                isObstacleFront = ~isempty(scanMsg) && ...
                                  detectObstacleFront(scanMsg, obstacleStopDistance);
    
                % 4c) If we have no global path, or we finished it, plan a new one
                if isempty(globalPath)
                    % Plan from currentPose(1:2) to [finalGoalX, finalGoalY]
                    disp('Planning a new global path with A*...');
                    globalPath = globalPlanAStar(occMap, currentPose, [finalGoalX, finalGoalY]);
                    currentWaypointIndex = 1;
                elseif currentWaypointIndex > size(globalPath,1)
                    % We reached the final waypoint
                    disp('Reached final goal. Stopping...');
                    cmdMsg.linear.x  = 0;
                    cmdMsg.angular.z = 0;
                    send(cmdVelPub, cmdMsg);
                    break;  % exit main loop
                end
    
                % 4d) Local Path Following
                %     We try to move from currentPose => next waypoint in globalPath
                nextWaypoint = globalPath(currentWaypointIndex,:);
    
                currentPose = maybeUpdatePose(odomSub, currentPose);
    
                % Check if we are close enough to next waypoint
                distToWP = norm([nextWaypoint(1)-currentPose(1), nextWaypoint(2)-currentPose(2)]);
                if distToWP < waypointThreshold
                    % Move on to next
                    currentWaypointIndex = currentWaypointIndex + 1;
                else
                    % If there's an obstacle, we do local re-plan
                    if isObstacleFront
                        disp('Local obstacle detected => local replan around it.');
                        % For local replan, we can do a smaller region A* or a short
                        % path from currentPose to the nextWaypoint:
                        localPath = localPlanAStar(occMap, currentPose, nextWaypoint);
                        if isempty(localPath)
                            disp('Local replan failed. Stopping...');
                            cmdMsg.linear.x  = 0;
                            cmdMsg.angular.z = 0;
                            send(cmdVelPub, cmdMsg);
                            break; % or try again later
                        else
                            % Replace the front portion of the path with localPath
                            globalPath = [localPath(1:end-1,:); globalPath(currentWaypointIndex:end,:)];
                            currentWaypointIndex = 1; % reset to start of new path
                        end
                    end
    
                    % Now do a simple move to nextWaypoint ignoring dynamic obstacle
                    [vLin, vAng] = simpleGoToPose(currentPose, nextWaypoint, ...
                                                  linearSpeedLimit, angularSpeedLimit);
                    % If front obstacle => force linear=0
                    if isObstacleFront
                        vLin = 0;
                    end
                    cmdMsg.linear.x  = vLin;
                    cmdMsg.angular.z = vAng;
                    send(cmdVelPub, cmdMsg);
                end
    
                % 4e) Visualization
                cla(ax);
                show(occMap, 'Parent', ax);
                drawRobot(ax, currentPose);
                if ~isempty(globalPath)
                    plot(ax, globalPath(:,1), globalPath(:,2), '-g', 'LineWidth',2);
                end
                plot(ax, finalGoalX, finalGoalY,'bo','LineWidth',2,'MarkerSize',8);
                drawnow limitrate;
    
                waitfor(rateCtrl);
            end
    
        catch ME
            disp('Exiting. Saving map as PGM + YAML...');
            saveMapAsImageAndYaml(occMap, "myMap.pgm", "myMap.yaml");
            rethrow(ME);
        end
    
        % Final cleanup
        disp('Done. Saving final map...');
        saveMapAsImageAndYaml(occMap, "myMap.pgm", "myMap.yaml");
    end
    
    
    %% ---------------------- HELPER FUNCTIONS --------------------------
    
    function [gx, gy] = getUser2DGoal()
    % Prompt user for 2D goal (x,y). No heading in this example.
        gx = str2double(input('Enter goal X (m): ','s'));
        gy = str2double(input('Enter goal Y (m): ','s'));
        fprintf('Goal => (%.2f, %.2f)\n',gx,gy);
    end
    
    
    function updateMap(occMap, scanMsg, robotPose, maxRange)
    % Insert rays with default thresholds
        [ranges, angles] = extractScan(scanMsg);
        valid = isfinite(ranges);
        ranges(~valid) = [];
        angles = angles(valid);
        if ~isempty(ranges)
            insertRay(occMap, robotPose, ranges, angles, maxRange);
        end
    end
    
    
    function [ranges, angles] = extractScan(scanMsg)
    % Extract numeric arrays from LaserScan
        angleMin = double(scanMsg.angle_min);
        angleInc = double(scanMsg.angle_increment);
        n = numel(scanMsg.ranges);
    
        anglesVec = angleMin + (0:(n-1))*angleInc;
        rangesVec = double(scanMsg.ranges);
    
        ranges = rangesVec(:)';
        angles = anglesVec(:)';
    end
    
    
    function isObstacle = detectObstacleFront(scanMsg, stopDist)
    % Check ±90 deg around forward direction
        [ranges,~] = extractScan(scanMsg);
        if isempty(ranges)
            isObstacle = false; 
            return;
        end
        n = numel(ranges);
        leftIdx  = floor(n/4); 
        rightIdx = floor(3*n/4);
    
        vranges = ranges; 
        invalid = ~isfinite(vranges) | (vranges <= 0);
        vranges(invalid) = inf;
        distFront = min([vranges(1:leftIdx), vranges(rightIdx+1:end)]);
        isObstacle = (distFront < stopDist);
    end
    
    
    function pose = maybeUpdatePose(odomSub, oldPose)
    % Try reading from /odom. If empty, keep old pose
        odomMsg = odomSub.LatestMessage;
        if ~isempty(odomMsg)
            pose = getPoseFromOdom(odomMsg);
        else
            pose = oldPose;
        end
    end
    
    
    function pose = getPoseFromOdom(odomMsg)
    % Return [x, y, yaw]
        p = odomMsg.pose.pose.position;
        x = double(p.x); y = double(p.y);
        q = odomMsg.pose.pose.orientation;
        orientation = [q.w, q.x, q.y, q.z];
        yaw = quat2yaw(orientation);
        pose = [x, y, yaw];
    end
    
    
    function yaw = quat2yaw(q)
        w=q(1); x=q(2); y=q(3); z=q(4);
        siny_cosp = 2*(w*z + x*y);
        cosy_cosp = 1 - 2*(y^2 + z^2);
        yaw = atan2(siny_cosp, cosy_cosp);
    end
    
    
    %% -------------------- GLOBAL PLANNER (A*) -------------------------
    
    function pathWorld = globalPlanAStar(occMap, startPose, goalXY)
    % Convert to binaryOccupancyMap, run A* from start => goal
        binMap = convertOccMapToBinary(occMap, 0.5);
    
        planner = plannerAStarGrid(binMap);
        %planner.ConnectionDistance = 1;  % tune as needed
        planner.DiagonalSearch = 'off';  % or true if you want 8-connected grid
    
        startWorld = startPose(1:2);  % [x,y]
        goalWorld  = goalXY;          % [x,y]
    
        % Convert world coords => grid coords (row, col),
        % which might be fractional if the points do not land exactly on cell centers.
        startGrid = world2grid(binMap, startWorld(1), startWorld(2));
        goalGrid  = world2grid(binMap, goalWorld(1), goalWorld(2));

        % Round them to integer row-col
        startGrid = round(startGrid);
        goalGrid  = round(goalGrid);

        % 3) A* plan in grid coordinates
        %    pathGrid is N-by-2, each row = [row, col].
        pathGrid = plan(planner, startGrid, goalGrid);

        % If empty => no feasible path
        if isempty(pathGrid)
            pathWorld = [];
            return;
        end

        % 4) Convert each [row, col] in pathGrid -> [x, y] in world coords
        n = size(pathGrid, 1);
        pathWorld = zeros(n, 2);
        for i = 1:n
            [wx, wy] = grid2world(binMap, pathGrid(i,1), pathGrid(i,2));
            pathWorld(i,:) = [wx, wy];
        end
    end
    
    
    %% -------------------- LOCAL PLANNER (A*) --------------------------
    function path = localPlanAStar(occMap, startPose, goalXY)
    % A smaller re-plan that can handle local obstacles
    % For demonstration, we just do the same A* over the entire map, but you
    % could limit it to a local region or do a different approach.
    
        path = globalPlanAStar(occMap, startPose, goalXY);
    end
    
    
    %% -------------------- Simple Path Follower ------------------------
    function [vLin, vAng] = simpleGoToPose(robotPose, targetXY, linLimit, angLimit)
    % A naive "go straight to waypoint" controller
    
        rx = robotPose(1); ry = robotPose(2); rth = robotPose(3);
        tx = targetXY(1);  ty = targetXY(2);
    
        dx = tx - rx;
        dy = ty - ry;
    
        dist = hypot(dx, dy);
        desiredHeading = atan2(dy, dx);
        headingError = wrapToPi(desiredHeading - rth);
    
        % Angular velocity
        vAng = headingError; 
        if abs(vAng) > angLimit
            vAng = angLimit * sign(vAng);
        end
    
        % Linear velocity
        vLin = dist * 0.5;  % scale factor
        if vLin > linLimit
            vLin = linLimit;
        end
    end
    
    
    %% -------------------- Convert Occupancy Maps ----------------------
    function binMap = convertOccMapToBinary(occMap, threshold)
        mat = occupancyMatrix(occMap); % values in [0..1]
        bin = (mat > threshold);       % 1=occupied, 0=free
        binMap = binaryOccupancyMap(bin, occMap.Resolution);
        binMap.LocalOriginInWorld = occMap.LocalOriginInWorld;
    end
    
    
    %% -------------------- Save Map as PGM + YAML ----------------------
    function saveMapAsImageAndYaml(occMap, pgmFile, yamlFile)
    % Convert occupancyMap => binaryOccupancyMap => nav_msgs/OccupancyGrid => .pgm + .yaml
        disp('Saving occupancyMap to .pgm + .yaml...');
        binMap = convertOccMapToBinary(occMap, 0.5);
    
        msg = ros2message("nav_msgs/OccupancyGrid");
        msgOut = rosWriteBinaryOccupancyGrid(msg, binMap);
    
        w = double(msgOut.info.width);
        h = double(msgOut.info.height);
        data = double(msgOut.data);
    
        if numel(data) ~= w*h
            warning('Occupancy data mismatch. Cancel saving.');
            return;
        end
    
        gridMat = reshape(data, [w,h])';
        % 0=free => white => 255, 100=occupied => black =>0, -1=> unknown =>205
        pgmMat = uint8(ones(size(gridMat))*205);
        isFree = (gridMat==0);
        isOcc  = (gridMat==100);
        pgmMat(isFree) = 254;
        pgmMat(isOcc)  = 0;
    
        imwrite(pgmMat, pgmFile,'pgm');
        disp("Wrote PGM: "+pgmFile);
    
        res = msgOut.info.resolution;
        ox  = msgOut.info.origin.position.x;
        oy  = msgOut.info.origin.position.y;
        q   = msgOut.info.origin.orientation;
        yaw = quat2yaw([q.w,q.x,q.y,q.z]);
    
        fid = fopen(yamlFile,'w');
        if fid<0
            warning('Cannot open YAML file: %s', yamlFile);
            return;
        end
        fprintf(fid,"image: %s\n", pgmFile);
        fprintf(fid,"resolution: %.6f\n", res);
        fprintf(fid,"origin: [%.6f, %.6f, %.6f]\n", ox, oy, yaw);
        fprintf(fid,"negate: 0\n");
        fprintf(fid,"occupied_thresh: 0.65\n");
        fprintf(fid,"free_thresh: 0.196\n");
        fclose(fid);
    
        disp("Wrote YAML: "+yamlFile);
    end
    