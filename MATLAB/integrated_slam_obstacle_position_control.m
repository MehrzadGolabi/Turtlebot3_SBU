function integrated_slam_obstacle_position_control()
    %INTEGRATED_SLAM_OBSTACLE_POSITION_CONTROL
    %
    % A single MATLAB node that:
    %  1) Builds a dynamic occupancy map from /scan + /odom.
    %  2) Detects obstacles in front and stops if too close.
    %  3) Uses simple position control to reach a user goal.
    %  4) Publishes /cmd_vel accordingly.
    %
    % Subscribes:
    %   /scan  (sensor_msgs/LaserScan)
    %   /odom  (nav_msgs/Odometry)
    %
    % Publishes:
    %   /cmd_vel (geometry_msgs/Twist)
    %
    % Steps:
    %  1) Start node, create subs/pubs
    %  2) Ask user for goal pose
    %  3) On each loop iteration:
    %      - Get LIDAR => update occupancy map => also do front obstacle detection
    %      - Get odometry => compute position-control velocity
    %      - Combine with obstacle detection => final velocity
    %      - Publish /cmd_vel
    %      - Plot map & robot
    %  4) On stop => optionally save map as PGM + YAML (like ROS 1).
    %
    % *** This is a conceptual integration. You may want to refine
    %     concurrency, code structure, or separate tasks into distinct nodes.
    
        %---------------------------%
        % 1. Setup Node & Topics
        %---------------------------%
        setenv('ROS_DOMAIN_ID','30');  % or your matching domain
        node = ros2node("/integrated_mapping_obstacle_control");  
    
        scanSub    = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
        odomSub    = ros2subscriber(node,"/odom","nav_msgs/Odometry");
        cmdVelPub  = ros2publisher(node,"/cmd_vel","geometry_msgs/Twist");
    
        % Pre-allocate a Twist for final command
        cmdMsg = ros2message("geometry_msgs/Twist");
    
        %---------------------------%
        % Occupancy Map Setup
        %---------------------------%
        mapSize       = 20;   % meters in X,Y
        mapResolution = 20;   % cells/m => 0.05m per cell
        occMap = occupancyMap(mapSize, mapSize, mapResolution);
        occMap.LocalOriginInWorld = [-10, -10]; % shift so the map covers [-10..+10, -10..+10]
    
        maxLidarRange = 3.5;  % typical LIDAR range on TB3
        stopDistance  = 0.5;  % obstacle detection threshold
    
        % Visualization
        figure('Name','SLAM + Obstacle + PositionControl');
        ax = axes(); hold on; grid on; axis equal;
        xlabel('X (m)'); ylabel('Y (m)');
        title('Dynamic Occupancy Map + Robot + Goal');
    
        %---------------------------%
        % Position Control Params
        %---------------------------%
        linearSpeedLimit  = 0.1;  % m/s max
        angularSpeedLimit = 1.5;  % rad/s max
        distanceThreshold = 0.05; % close enough in position
        headingThreshold  = deg2rad(1.0); % close enough in orientation
    
        %---------------------------%
        % 2. Prompt User for Goal
        %---------------------------%
        [goalX, goalY, goalHeading] = getUserGoal();
    
        %---------------------------%
        % 3. Main Loop
        %---------------------------%
        loopRate = ros2rate(node, 5);  % run ~5 Hz
    
        currentRobotPose = [0 0 0];  % [x, y, theta]
        isGoalReached = false;
    
        try
            while true
                % (a) Retrieve the latest scan
                scanMsg = scanSub.LatestMessage;
                if ~isempty(scanMsg)
                    % Update occupancy map
                    occMap = updateMap(occMap, scanMsg, currentRobotPose, maxLidarRange);
                end
    
                % Also do obstacle detection in front sector
                isObstacle = false;
                if ~isempty(scanMsg)
                    isObstacle = detectObstacleFront(scanMsg, stopDistance);
                end
    
                % (b) Retrieve the latest odometry => current pose
                odomMsg = odomSub.LatestMessage;
                if ~isempty(odomMsg)
                    currentRobotPose = getPoseFromOdom(odomMsg);
                end
    
                % (c) Position control => desired velocity
                if ~isGoalReached
                    [vLin, vAng, isGoalReached] = ...
                        positionControl(currentRobotPose, ...
                                        [goalX, goalY, goalHeading], ...
                                        linearSpeedLimit, angularSpeedLimit, ...
                                        distanceThreshold, headingThreshold);
                else
                    vLin = 0.0; vAng = 0.0;
                end
    
                % (d) Combine with obstacle detection override
                if isObstacle
                    vLin = 0.0;  % zero linear if obstacle is too close
                end
    
                % (e) Publish final velocity
                cmdMsg.linear.x  = vLin;
                cmdMsg.angular.z = vAng;
                send(cmdVelPub, cmdMsg);
    
                % (f) Visualize
                cla(ax);
                show(occMap, 'Parent', ax);
                drawGoal(ax, [goalX, goalY]);
                drawRobot(ax, currentRobotPose);
    
                drawnow limitrate;
    
                % If goal is reached, ask the user if they want a new goal
                if isGoalReached
                    disp('Goal reached! Enter new goal or press Ctrl+C to stop.');
                    [goalX, goalY, goalHeading] = getUserGoal();
                    isGoalReached = false;
                end
    
                waitfor(loopRate);
            end
    
        catch ME
            % On Ctrl+C or error, optionally save the final map
            disp('Stopping. Saving final map as PGM + YAML...');
            saveMapAsImageAndYaml(occMap, "myMap.pgm", "myMap.yaml");
            rethrow(ME);
        end
    end
    
    
    %% -------------------- HELPER FUNCTIONS -------------------------------
    
    function [gx, gy, gheading] = getUserGoal()
    % Ask user for goal (X, Y, Heading in degrees), convert to radians
        gx = str2double(input('Enter goal X (m): ','s'));
        gy = str2double(input('Enter goal Y (m): ','s'));
        hdgDeg = str2double(input('Enter goal Heading (deg): ','s'));
        gheading = deg2rad(hdgDeg);
    
        % Wrap heading to [-pi, pi]
        gheading = wrapToPi(gheading);
    
        fprintf('New Goal => X=%.2f, Y=%.2f, Heading=%.2f deg\n',...
                gx, gy, rad2deg(gheading));
    end
    
    
    function occMap = updateMap(occMap, scanMsg, robotPose, maxRange)
    % Insert the new scan into the occupancyMap for dynamic obstacle updates
    
        [ranges, angles] = extractLaserData(scanMsg);
        valid = isfinite(ranges);
        ranges(~valid) = [];
        angles = angles(valid);
    
        if ~isempty(ranges)
            insertRay(occMap, robotPose, ranges, angles, maxRange);
        end
    end
    
    
    function isObstacle = detectObstacleFront(scanMsg, stopDistance)
    % A minimal front-obstacle check in ±90 degrees from the forward direction
    % If any reading < stopDistance => isObstacle = true
    
        [ranges, ~] = extractLaserData(scanMsg);
        if isempty(ranges)
            isObstacle = false; return;
        end
    
        n = numel(ranges);
        leftIdx  = floor(n/4);    % ~90 deg
        rightIdx = floor(3*n/4);  % ~270 deg
    
        validRanges = ranges;
        invalid = ~isfinite(validRanges) | (validRanges<=0);
        validRanges(invalid) = inf;
    
        distFront = min([validRanges(1:leftIdx), validRanges(rightIdx+1:end)]);
    
        isObstacle = (distFront < stopDistance);
    end
    
    
    function [ranges, angles] = extractLaserData(scanMsg)
    % Convert LaserScan => numeric row vectors
        angleMin = double(scanMsg.angle_min);
        angleInc = double(scanMsg.angle_increment);
        n = numel(scanMsg.ranges);
    
        anglesVec = angleMin + (0:(n-1))*angleInc;
        rangesVec = double(scanMsg.ranges);
    
        ranges = rangesVec(:)';
        angles = anglesVec(:)';
    end
    
    
    function pose = getPoseFromOdom(odomMsg)
    % Extract [x, y, theta] from nav_msgs/Odometry
        p = odomMsg.pose.pose.position;
        x = double(p.x);
        y = double(p.y);
    
        q = odomMsg.pose.pose.orientation;
        orientation = [q.w, q.x, q.y, q.z];
        yaw = quat2yaw(orientation);
    
        pose = [x, y, yaw];
    end
    
    
    function yaw = quat2yaw(quat)
    % Convert quaternion [w x y z] => yaw in [-pi, pi]
        w = quat(1); x = quat(2); y = quat(3); z = quat(4);
        siny_cosp = 2.0*(w*z + x*y);
        cosy_cosp = 1.0 - 2.0*(y*y + z*z);
        yaw = atan2(siny_cosp, cosy_cosp);
    end
    
    
    function [vLin, vAng, isGoalReached] = positionControl(robotPose, goalPose, ...
                                         linLimit, angLimit, distThresh, hdgThresh)
    % Simple position control: Move to (goalX, goalY), then rotate to goalHeading.
    % Return linear speed (vLin), angular speed (vAng), and isGoalReached (bool).
    
        % Unpack
        rx = robotPose(1); ry = robotPose(2); rth = robotPose(3);
        gx = goalPose(1);  gy = goalPose(2);  gth = goalPose(3);
    
        dx = gx - rx;
        dy = gy - ry;
        dist = hypot(dx, dy);
    
        % If not close enough in position => move
        if dist > distThresh
            % angle to goal
            desiredHeading = atan2(dy, dx);
            headingError = wrapToPi(desiredHeading - rth);
    
            % Angular velocity ~ heading error
            vAng = headingError;
            vAng = clamp(vAng, -angLimit, angLimit);
    
            % Linear velocity scaled by distance, but limit
            vLin = 0.5 * dist;
            vLin = min(vLin, linLimit);
    
            isGoalReached = false;
            return;
        else
            % Position is close => fix heading
            hdgError = wrapToPi(gth - rth);
            if abs(hdgError) > hdgThresh
                % Turn in place
                turnSpeed = clamp(1.0*hdgError, -angLimit, angLimit);
                vLin = 0.0;
                vAng = turnSpeed;
                isGoalReached = false;
            else
                % We have reached position + heading
                vLin = 0.0; 
                vAng = 0.0;
                isGoalReached = true;
            end
        end
    end
    
    
    function val = clamp(x, mn, mx)
    %CLAMP Limit x to [mn, mx].
        val = min(max(x, mn), mx);
    end
    
    
    function drawRobot(ax, pose)
    % Draw a small triangular marker
        x = pose(1); y = pose(2); th = pose(3);
        r = 0.2;
        pts = [ r,   0;
               -r,  0.5*r;
               -r, -0.5*r;
                r,   0 ];
        R = [cos(th), -sin(th); sin(th), cos(th)];
        ptsG = (R*pts')';
        ptsG(:,1) = ptsG(:,1)+x;
        ptsG(:,2) = ptsG(:,2)+y;
    
        patch(ax,'XData',ptsG(:,1),'YData',ptsG(:,2), ...
              'FaceColor','r','EdgeColor','k');
    end
    
    
    function drawGoal(ax, goalPose)
    % Draw a small circle for the goal
        gx = goalPose(1);
        gy = goalPose(2);
        plot(ax, gx, gy, 'go','MarkerSize',10,'LineWidth',2);
    end
    
    
    function saveMapAsImageAndYaml(occMap, pgmFile, yamlFile)
    % Optionally called on exit to store the final occupancyMap => PGM + YAML
    % For standard ROS usage, we must produce a binaryOccupancyMap and then
    % use rosWriteBinaryOccupancyGrid, etc.
    
        disp('Converting occupancyMap => binaryOccupancyMap => PGM + YAML...');
        % Convert
        threshold = 0.5;
        binMap = convertOccMapToBinaryMap(occMap, threshold);
    
        % Make nav_msgs/OccupancyGrid message
        mapMsg = ros2message("nav_msgs/OccupancyGrid");
        msgOut = rosWriteBinaryOccupancyGrid(mapMsg, binMap);
    
        % Reshape data => image => .pgm
        width  = double(msgOut.info.width);
        height = double(msgOut.info.height);
        occData = double(msgOut.data);
    
        if numel(occData) ~= (width*height)
            warning("Occupancy data mismatch. Not saving.");
            return;
        end
    
        gridMat = reshape(occData, [width, height])';
    
        % 0=free => white(255), 100=occupied => black(0), -1 => unknown(205)
        pgmMat = uint8(ones(size(gridMat))*205);
        isFree = (gridMat==0);
        isOcc  = (gridMat==100);
        pgmMat(isFree) = 254; 
        pgmMat(isOcc)  = 0;
    
        % (Optional) flipud(pgmMat) if needed
    
        imwrite(pgmMat, pgmFile,'pgm');
        disp("Wrote PGM: " + pgmFile);
    
        % YAML
        resolution = msgOut.info.resolution;
        ox = msgOut.info.origin.position.x;
        oy = msgOut.info.origin.position.y;
        q = msgOut.info.origin.orientation;
        yaw = quat2yaw([q.w, q.x, q.y, q.z]);
    
        fid = fopen(yamlFile,'w');
        if fid<0
            warning("Cannot write YAML: " + yamlFile);
            return;
        end
        fprintf(fid,"image: %s\n", pgmFile);
        fprintf(fid,"resolution: %.6f\n", resolution);
        fprintf(fid,"origin: [%.6f, %.6f, %.6f]\n", ox, oy, yaw);
        fprintf(fid,"negate: 0\n");
        fprintf(fid,"occupied_thresh: 0.65\n");
        fprintf(fid,"free_thresh: 0.196\n");
        fclose(fid);
    
        disp("Wrote YAML: " + yamlFile);
    end
    
    
    function binMap = convertOccMapToBinaryMap(occMap, threshold)
    % Convert continuous occupancyMap => binaryOccupancyMap (required by rosWriteBinaryOccupancyGrid)
        mat = occupancyMatrix(occMap); % [0..1]
        bin = (mat > threshold); % logical
        binMap = binaryOccupancyMap(bin, occMap.Resolution);
        binMap.LocalOriginInWorld = occMap.LocalOriginInWorld;
    end
    