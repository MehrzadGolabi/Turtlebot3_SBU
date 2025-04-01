function live_slam_overlay()
    setenv('ROS_DOMAIN_ID','30');

    node = ros2node("/live_slam_overlay");
    scanSub = ros2subscriber(node,"/scan","sensor_msgs/LaserScan");
    odomSub = ros2subscriber(node,"/odom","nav_msgs/Odometry");

    %% 1) Load old map + parse YAML
    mapPgm  = "myMap.pgm";
    mapYaml = "myMap.yaml";
    if ~isfile(mapPgm) || ~isfile(mapYaml)
        error("Cannot find %s or %s. Please run dynamic_occupancy_mapping.m first.",mapPgm,mapYaml);
    end

    [mapRes, originData] = parseMapYaml(mapYaml);
    oldOx = originData(1);
    oldOy = originData(2);
    oldYaw= originData(3);  % might be 0.0

    % If oldOx=-6.66, oldOy=-6.27 => we want liveMapOffset = [ +6.66, +6.27, 0 ]
    liveMapOffset = [ -oldOx, -oldOy, 0];  % negative sign => "inverse" shift

    baseImg = imread(mapPgm);
    occMask = (baseImg < 128);  % 1=occ
    staticMap = binaryOccupancyMap(occMask, 1/mapRes);
    % No shift => it appears at [0..width, 0..height] in axes
    % (Alternatively you could do staticMap.LocalOriginInWorld=[0,0], up to you.)

    %% 2) Create new map that we continuously update
    initSize = 5;  % smaller, can expand if needed
    liveMapRes = 20; % same as old map? or 20 if you want
    liveMap = occupancyMap(initSize, initSize, liveMapRes);

    canAutoExpand = false;
    try
        liveMap.ExceedingMapBoundsAction = 'expand';
        canAutoExpand = true;
    catch
        warning("No auto-expand => fallback manual expand");
    end

    % We set no shift for liveMap itself => [0,0]. We'll handle shifting
    % in the code by 'liveMapOffset' each time.
    liveMap.LocalOriginInWorld = [0,0];

    maxLidarRange = 3.5;

    %% 3) Plot the static map as background
    fig = figure('Name','Live SLAM Overlay (Negative Offset from YAML)');
    ax  = axes('Parent',fig);
    hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
    title(ax,'Static Map + New Map Overlay. Offset = negative of old origin.');
    xlabel(ax,'X (pixels or meters)'); ylabel(ax,'Y (pixels or meters)');

    show(staticMap,'Parent',ax);
    drawnow;

    r = ros2rate(node,5);
    disp("Overlay started. Press Ctrl+C to stop...");

    overlayHandle   = [];
    robotPoseHandle = [];

    %% 4) Main loop
    try
        while true
            scanMsg = scanSub.LatestMessage;
            odomMsg = odomSub.LatestMessage;
            if isempty(scanMsg) || isempty(odomMsg)
                waitfor(r);
                continue;
            end

            [ranges, angles] = extractLaserData(scanMsg);
            ranges(isinf(ranges)) = maxLidarRange;

            odomPose = getPoseFromOdom(odomMsg);

            % We SHIFT by liveMapOffset => effectively aligning with the old map
            mapPose  = applyPoseOffset(odomPose, liveMapOffset);

            if ~canAutoExpand
                liveMap = expandMapIfNeeded(liveMap, mapPose, maxLidarRange);
            end

            insertRay(liveMap, mapPose, ranges, angles, maxLidarRange);

            % Visualization
            if ~isvalid(ax) || ~ishghandle(ax) || ~strcmpi(get(ax,'type'),'axes')
                fig=figure('Name','Live SLAM Overlay (Recreated)');
                ax=axes('Parent',fig); hold(ax,'on'); axis(ax,'equal'); grid(ax,'on');
                show(staticMap,'Parent',ax);
                overlayHandle=[];
                robotPoseHandle=[];
            end

            if ishandle(overlayHandle),   delete(overlayHandle);   end
            if ishandle(robotPoseHandle), delete(robotPoseHandle); end

            occMat = occupancyMatrix(liveMap);
            occMaskNew = (occMat>0.65);

            [mh,nw] = size(occMat);
            liveRGB = zeros(mh,nw,3,'double');
            liveRGB(:,:,1) = occMaskNew;  % red
            liveRGB(:,:,3) = occMaskNew;  % blue

            liveRGBFlip = flipud(liveRGB);
            alphaFlip   = flipud(occMaskNew);

            xLims = liveMap.XWorldLimits;
            yLims = liveMap.YWorldLimits;

            overlayHandle = imagesc(ax, xLims, yLims, liveRGBFlip);
            set(overlayHandle,'AlphaData',0.4*alphaFlip);

            robotPoseHandle = drawRobot(ax, mapPose, 'm');

            drawnow limitrate;
            waitfor(r);
        end
    catch ME
        disp("Overlay stopped.");
        rethrow(ME);
    end
end

%% ---------------- HELPER: parseMapYaml ----------------
function [res, origin] = parseMapYaml(yamlFile)
    res=1.0; origin=[0,0,0];
    fid=fopen(yamlFile,'r');
    if fid<0
        warning("Cannot open YAML: %s",yamlFile);
        return;
    end
    raw=textscan(fid,'%s','Delimiter','\n');
    fclose(fid);
    lines=raw{1};
    for i=1:numel(lines)
        L=strtrim(lines{i});
        if startsWith(L,'resolution:')
            parts=strsplit(L,':');
            res=str2double(strtrim(parts{2}));
        elseif startsWith(L,'origin:')
            parts=strsplit(L,':');
            val=strrep(strrep(strtrim(parts{2}),'[',''),']','');
            coords=strsplit(val,',');
            if numel(coords)==3
                ox = str2double(coords{1});
                oy = str2double(coords{2});
                yaw= str2double(coords{3});
                origin=[ox,oy,yaw];
            end
        end
    end
end

%% ---------------- HELPER: getPoseFromOdom ----------------
function pose = getPoseFromOdom(odomMsg)
    p=odomMsg.pose.pose.position;
    x=double(p.x); y=double(p.y);
    q=odomMsg.pose.pose.orientation;
    orientation=[q.w,q.x,q.y,q.z];
    yaw=quat2yaw(orientation);
    pose=[x,y,yaw];
end

function yaw=quat2yaw(quat)
    w=quat(1); x=quat(2); y=quat(3); z=quat(4);
    siny_cosp=2*(w*z + x*y);
    cosy_cosp=1-2*(y^2+z^2);
    yaw=atan2(siny_cosp,cosy_cosp);
end

%% ---------------- HELPER: extractLaserData ----------------
function [ranges, angles] = extractLaserData(scanMsg)
    angleMin = double(scanMsg.angle_min);
    angleInc = double(scanMsg.angle_increment);
    n= numel(scanMsg.ranges);
    anglesVec= angleMin + (0:(n-1))*angleInc;
    rangesVec= double(scanMsg.ranges);

    valid=isfinite(rangesVec) | isinf(rangesVec);
    ranges=rangesVec(valid); angles=anglesVec(valid);
end

%% ---------------- HELPER: applyPoseOffset ----------------
function mapPose = applyPoseOffset(odomPose, offsetVec)
    % offsetVec = [dx, dy, dtheta], ignoring dtheta if you don't need rotation.
    dx= offsetVec(1);
    dy= offsetVec(2);
    dth= offsetVec(3);

    xW = odomPose(1) + dx;
    yW = odomPose(2) + dy;
    thW= odomPose(3) + dth;
    mapPose=[xW,yW,thW];
end

%% ---------------- HELPER: expandMapIfNeeded ----------------
function occMap=expandMapIfNeeded(occMap,mapPose,maxRange)
    x=mapPose(1); y=mapPose(2);

    xlim=occMap.XWorldLimits;
    ylim=occMap.YWorldLimits;

    margin=maxRange+1;
    needXmin=x-margin; needXmax=x+margin;
    needYmin=y-margin; needYmax=y+margin;

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

    newWidth= newXmax-newXmin;
    newHeight=newYmax-newYmin;
    newOccMap=occupancyMap(newWidth,newHeight,oldRes);
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
    disp('Map expanded.');
end

function h = drawRobot(ax, pose, colorVal)
    x=pose(1); y=pose(2); th=pose(3);
    r=0.2;
    pts=[ r,0; -r,0.5*r; -r,-0.5*r; r,0 ];
    R=[cos(th),-sin(th); sin(th),cos(th)];
    ptsG=(R*pts')';
    ptsG(:,1)=ptsG(:,1)+x;
    ptsG(:,2)=ptsG(:,2)+y;
    h=patch(ax,'XData',ptsG(:,1),'YData',ptsG(:,2),...
         'FaceColor',colorVal,'EdgeColor','k','FaceAlpha',0.6);
end
