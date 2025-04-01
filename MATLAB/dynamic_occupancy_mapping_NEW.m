function dynamic_occupancy_mapping()
    setenv('ROS_DOMAIN_ID','30');  % Force domain ID

    node = ros2node("/live_dynamic_mapping");
    scanSub = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
    odomSub = ros2subscriber(node,"/odom","nav_msgs/Odometry");

    %% 1) Create occupancyMap
    initSize   = 5;       % 5 m x 5 m
    resolution = 20;      % 0.05 m/cell
    occMap = occupancyMap(initSize, initSize, resolution);

    canAutoExpand = false;
    try
        occMap.ExceedingMapBoundsAction = 'expand';
        canAutoExpand = true;
    catch
        warning(['Map expansion not supported or triggered conflict. ', ...
                 'Falling back to manual expansion.']);
    end

    mapOriginInWorld = [0, 0];
    mapYawInWorld    = deg2rad(0);
    occMap.LocalOriginInWorld = mapOriginInWorld;

    maxLidarRange = 3.5;

    figure('Name','Dynamic Occupancy Mapping');
    ax = axes();
    hold(ax,'on'), grid(ax,'on'), axis(ax,'equal');
    title(ax,'Dynamic Occupancy Mapping');
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)');

    r = ros2rate(node,5);
    disp('Mapping started. Press Ctrl+C to stop...');

    try
        while true
            scanMsg = scanSub.LatestMessage;
            odomMsg = odomSub.LatestMessage;
            if isempty(scanMsg) || isempty(odomMsg)
                waitfor(r);
                continue;
            end

            odomPose = getPoseFromOdom(odomMsg);
            mapPose  = worldPoseToMapPose(odomPose, mapOriginInWorld, mapYawInWorld);

            [ranges, angles] = extractLaserData(scanMsg);
            ranges(isinf(ranges)) = maxLidarRange;  % large open spaces

            if ~canAutoExpand
                occMap = expandMapIfNeeded(occMap, mapPose, maxLidarRange);
            end

            insertRay(occMap, mapPose, ranges, angles, maxLidarRange);

            cla(ax);
            show(occMap,'Parent',ax);
            drawRobot(ax, mapPose);
            drawnow limitrate;

            waitfor(r);
        end
    catch ME
        disp('Stopping. Saving final map + .mat...');
        saveMapAsImageAndYaml(occMap, "myMap.pgm", "myMap.yaml", mapYawInWorld);
        rethrow(ME);
    end
end

%% ------------------ HELPER FUNCTIONS ------------------

function [ranges, angles] = extractLaserData(scanMsg)
    angleMin = double(scanMsg.angle_min);
    angleInc = double(scanMsg.angle_increment);
    n = numel(scanMsg.ranges);
    anglesVec = angleMin + (0:(n-1))*angleInc;
    rangesVec = double(scanMsg.ranges);

    valid = isfinite(rangesVec) | isinf(rangesVec);
    ranges = rangesVec(valid);
    angles = anglesVec(valid);
end

function pose = getPoseFromOdom(odomMsg)
    p = odomMsg.pose.pose.position;
    x = double(p.x);
    y = double(p.y);
    q = odomMsg.pose.pose.orientation;
    orientation = [q.w, q.x, q.y, q.z];
    theta = quat2yaw(orientation);
    pose = [x, y, theta];
end

function yaw = quat2yaw(quat)
    w=quat(1); x=quat(2); y=quat(3); z=quat(4);
    siny_cosp=2*(w*z + x*y);
    cosy_cosp=1-2*(y^2+z^2);
    yaw=atan2(siny_cosp,cosy_cosp);
end

function mapPose = worldPoseToMapPose(odomPose, mapOrigin, mapYaw)
    xW = odomPose(1) - mapOrigin(1);
    yW = odomPose(2) - mapOrigin(2);
    thW= odomPose(3);

    c=cos(mapYaw); s=sin(mapYaw);
    xM= xW*c + yW*s;
    yM=-xW*s + yW*c;
    thM= thW + mapYaw;
    mapPose=[xM,yM,thM];
end

function drawRobot(ax, pose)
    x=pose(1); y=pose(2); th=pose(3);
    r=0.2;
    pts=[ r,0; -r,0.5*r; -r,-0.5*r; r,0];
    R=[cos(th),-sin(th); sin(th),cos(th)];
    ptsG=(R*pts')';
    ptsG(:,1)=ptsG(:,1)+x;
    ptsG(:,2)=ptsG(:,2)+y;
    patch(ax,'XData',ptsG(:,1),'YData',ptsG(:,2),'FaceColor','r','EdgeColor','k');
end

function saveMapAsImageAndYaml(occMap, pgmFile, yamlFile, mapYaw)
    %% 1) Convert to binary => .pgm + .yaml as usual
    threshold=0.5;
    binMap=convertOccMapToBinaryMap(occMap,threshold);

    msg=ros2message("nav_msgs/OccupancyGrid");
    msgOut=rosWriteBinaryOccupancyGrid(msg,binMap);

    width = double(msgOut.info.width);
    height=double(msgOut.info.height);
    occData=double(msgOut.data);
    if numel(occData)~=width*height
        warning("Mismatch in occupancy data size vs width*height.");
        return;
    end
    gridMat=reshape(occData,[width,height])';
    pgmMat=uint8(ones(size(gridMat))*205);
    isFree=(gridMat==0);
    isOcc =(gridMat==100);
    pgmMat(isFree)=254;
    pgmMat(isOcc)=0;

    imwrite(pgmMat, pgmFile,'pgm');
    disp("Wrote PGM file: "+pgmFile);

    resolution=msgOut.info.resolution;
    ox=occMap.LocalOriginInWorld(1);
    oy=occMap.LocalOriginInWorld(2);

    fid=fopen(yamlFile,'w');
    if fid<0
        warning('Cannot open: %s',yamlFile);
        return;
    end
    fprintf(fid,"image: %s\n",pgmFile);
    fprintf(fid,"resolution: %.6f\n",resolution);
    fprintf(fid,"origin: [%.6f, %.6f, %.6f]\n",ox, oy, mapYaw);
    fprintf(fid,"negate: 0\n");
    fprintf(fid,"occupied_thresh: 0.65\n");
    fprintf(fid,"free_thresh: 0.196\n");
    fclose(fid);
    disp("Wrote YAML file: "+yamlFile);

    %% 2) Also save a .mat file for matchScansGrid
    % We'll store the final occupancy matrix in double [0..1] form.
    finalOccMatrix = occupancyMatrix(occMap);  % double in [0..1]
    finalResolution= occMap.Resolution;        % cells/m
    finalOrigin   = occMap.LocalOriginInWorld; % [ox, oy]
    finalYaw      = mapYaw;                    % store your mapYaw

    matFile = "myMapGrid.mat";
    save(matFile, "finalOccMatrix", "finalResolution", "finalOrigin", "finalYaw");
    disp("Saved .mat for matchScansGrid: "+matFile);
end

function binMap=convertOccMapToBinaryMap(occMap, threshold)
    mat=occupancyMatrix(occMap);
    bin=(mat>threshold);
    binMap=binaryOccupancyMap(bin, occMap.Resolution);
    binMap.LocalOriginInWorld=occMap.LocalOriginInWorld;
end

function occMap = expandMapIfNeeded(occMap, mapPose, maxRange)
    x=mapPose(1); y=mapPose(2);
    xlim=occMap.XWorldLimits;
    ylim=occMap.YWorldLimits;
    margin=maxRange+1;
    needXmin = x - margin;  needXmax = x + margin;
    needYmin = y - margin;  needYmax = y + margin;

    newXmin=xlim(1); newXmax=xlim(2);
    newYmin=ylim(1); newYmax=ylim(2);

    expandNeeded=false;
    if needXmin<xlim(1), newXmin=needXmin; expandNeeded=true; end
    if needXmax>xlim(2), newXmax=needXmax; expandNeeded=true; end
    if needYmin<ylim(1), newYmin=needYmin; expandNeeded=true; end
    if needYmax>ylim(2), newYmax=needYmax; expandNeeded=true; end

    if ~expandNeeded
        return;
    end

    oldMat=occupancyMatrix(occMap);
    oldSize=size(oldMat);
    oldRes=occMap.Resolution;
    oldOrg=occMap.LocalOriginInWorld;

    newWidth= newXmax - newXmin;
    newHeight=newYmax - newYmin;
    newOccMap=occupancyMap(newWidth, newHeight, oldRes);
    newOccMap.LocalOriginInWorld=[newXmin,newYmin];

    [rowIdx,colIdx]=ndgrid(1:oldSize(1),1:oldSize(2));
    rowIdx=rowIdx(:); colIdx=colIdx(:);
    IJ=[rowIdx,colIdx];
    worldXY=grid2world(occMap,IJ);
    oldVals=oldMat(sub2ind(oldSize,rowIdx,colIdx));
    for k=1:numel(oldVals)
        setOccupancy(newOccMap, worldXY(k,:), oldVals(k));
    end
    occMap=newOccMap;
    disp('Map manually expanded to fit region.');
end
