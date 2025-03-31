function dynamic_occupancy_mapping()
%DYNAMIC_OCCUPANCY_MAPPING
% Continuously builds an occupancyMap from a TurtleBot3 LIDAR (/scan) + odometry (/odom).
% Dynamic obstacles are removed if later scans see that space as free (continuous probabilities).
%
% On exit, the script converts the final 'occupancyMap' => 'binaryOccupancyMap' => 
% 'nav_msgs/OccupancyGrid' => .pgm + .yaml, using rosWriteBinaryOccupancyGrid.
%
% Subscribes:
%   /scan  (sensor_msgs/LaserScan)
%   /odom  (nav_msgs/Odometry)
%
% Creates an occupancyMap with LocalOriginInWorld offset, inserts rays in real time.
% Then stops and saves myMap.pgm + myMap.yaml.

    %% 1) Create ROS 2 node and subscribers
    node = ros2node("/live_dynamic_mapping");  % Node name in ROS 2
    scanSub = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
    odomSub = ros2subscriber(node,"/odom","nav_msgs/Odometry");

    %% 2) Initialize occupancyMap
    mapSize       = 20;    % meters in X and Y
    mapResolution = 20;    % cells/m => cell size 0.05 m
    occMap = occupancyMap(mapSize, mapSize, mapResolution);
    % Use LocalOriginInWorld (replaces the deprecated OriginInWorld):
    occMap.LocalOriginInWorld = [-10, -10];  % Shift so map covers [-10..+10, -10..+10]

    maxLidarRange = 3.5;  % typical TB3 range ~3.5 m

    figure('Name','Dynamic Occupancy Mapping');
    ax = axes();
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    title(ax,'Real-Time Dynamic Mapping');
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');

    %% 3) Main loop at ~5 Hz
    r = ros2rate(node,5);
    disp('Mapping started. Drive the robot around. Press Ctrl+C to stop...');

    currentRobotPose = [0,0,0];  % [x,y,theta]

    try
        while true
            % a) Read LaserScan
            scanMsg = scanSub.LatestMessage;
            if isempty(scanMsg)
                waitfor(r);
                continue;
            end

            % b) Read Odometry => robot pose
            odomMsg = odomSub.LatestMessage;
            if ~isempty(odomMsg)
                currentRobotPose = getPoseFromOdom(odomMsg);
            end

            % c) Extract ranges/angles from scan
            [ranges, angles] = extractLaserData(scanMsg);
            valid = isfinite(ranges);
            ranges(~valid) = [];
            angles = angles(valid);
            if isempty(ranges)
                waitfor(r);
                continue;
            end

            % d) Insert rays => dynamic updates
            insertRay(occMap, currentRobotPose, ranges, angles, ...
                      maxLidarRange);

            % e) Visualize
            cla(ax);
            show(occMap,'Parent',ax);
            drawRobot(ax, currentRobotPose);
            drawnow limitrate;

            waitfor(r);
        end
    catch ME
        % On Ctrl+C or error, finalize map => PGM + YAML
        disp('Stopping mapping. Saving the final map as PGM + YAML...');
        saveMapAsImageAndYaml(occMap, "myMap.pgm", "myMap.yaml");
        rethrow(ME);
    end
end


%% ----------- HELPER FUNCTIONS ---------------------------------------

function [ranges, angles] = extractLaserData(scanMsg)
% Convert LaserScan to numeric row vectors
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
    theta = quat2yaw(orientation);

    pose = [x,y,theta];
end


function yaw = quat2yaw(quat)
% Convert quaternion [w x y z] => yaw
    w = quat(1); x = quat(2); y = quat(3); z = quat(4);
    siny_cosp = 2.0*(w*z + x*y);
    cosy_cosp = 1.0 - 2.0*(y*y + z*z);
    yaw = atan2(siny_cosp, cosy_cosp);
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


function saveMapAsImageAndYaml(occMap, pgmFile, yamlFile)
% Convert from occupancyMap => binaryOccupancyMap => nav_msgs/OccupancyGrid => PGM + YAML

    % 1) Convert continuous occupancyMap to binaryOccupancyMap for rosWriteBinaryOccupancyGrid
    threshold = 0.5;  % any cell >0.5 => occupied, else free
    binMap = convertOccMapToBinaryMap(occMap, threshold);

    % 2) Create an empty OccupancyGrid message
    msg = ros2message("nav_msgs/OccupancyGrid");
    % Fill it using rosWriteBinaryOccupancyGrid
    msgOut = rosWriteBinaryOccupancyGrid(msg, binMap);

    % 3) Reshape msgOut.data into a 2D matrix
    width  = double(msgOut.info.width);
    height = double(msgOut.info.height);
    occData = double(msgOut.data); 
    if numel(occData) ~= width*height
        warning("Mismatch in occupancy data size vs width*height.");
        return;
    end
    gridMat = reshape(occData, [width, height])';

    % 'gridMat' is in [0..100] or -1 (unknown)
    % In a typical ROS OccupancyGrid:
    %   0=free, 100=occupied, -1=unknown
    % For .pgm: 0=black => occupied, 255=white => free, ~205 => unknown
    pgmMat = uint8(ones(size(gridMat)) * 205);  % unknown => 205
    isFree = (gridMat==0);
    isOcc  = (gridMat==100);
    % If you see an inverted map, swap isFree and isOcc usage as needed
    pgmMat(isFree) = 254;  % near white
    pgmMat(isOcc)  = 0;    % black

    % (Optional) Flip vertically if needed to match coordinate orientation:
    % pgmMat = flipud(pgmMat);

    % 4) Write PGM
    imwrite(pgmMat, pgmFile,'pgm');
    disp("Wrote PGM file: " + pgmFile);

    % 5) Write YAML
    % Use info in msgOut.info
    resolution = msgOut.info.resolution;
    ox = msgOut.info.origin.position.x;
    oy = msgOut.info.origin.position.y;
    q  = msgOut.info.origin.orientation;
    yaw = quat2yaw([q.w, q.x, q.y, q.z]);

    fid = fopen(yamlFile,'w');
    if fid<0
        warning('Cannot open YAML: %s', yamlFile);
        return;
    end
    fprintf(fid, "image: %s\n", pgmFile);
    fprintf(fid, "resolution: %.6f\n", resolution);
    fprintf(fid, "origin: [%.6f, %.6f, %.6f]\n", ox, oy, yaw);
    fprintf(fid, "negate: 0\n");
    fprintf(fid, "occupied_thresh: 0.65\n");
    fprintf(fid, "free_thresh: 0.196\n");
    fclose(fid);

    disp("Wrote YAML file: " + yamlFile);
end


function binMap = convertOccMapToBinaryMap(occMap, threshold)
% Convert a continuous-value occupancyMap to a binaryOccupancyMap using
% a chosen threshold in [0..1].
    mat  = occupancyMatrix(occMap); % returns values in [0..1]
    bin  = (mat > threshold);       % logical
    binMap = binaryOccupancyMap(bin, occMap.Resolution);
    % Copy local origin offset
    binMap.LocalOriginInWorld = occMap.LocalOriginInWorld;
end
