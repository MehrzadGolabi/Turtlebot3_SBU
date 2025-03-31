function check_nearest_obstacle()
% CHECK_NEAREST_OBSTACLE
% Continuously reads from /scan and prints the nearest obstacle distance in meters.

    % Create a ROS 2 node in MATLAB
    nodeName = "check_obstacle_node";
    setenv('ROS_DOMAIN_ID','30');
    node     = ros2node(nodeName,30);

    % Create a subscriber for LaserScan
    scanSub = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan");

    disp("Node created. Listening to /scan for nearest obstacle data...");
    disp("Press Ctrl+C or stop the script to exit.");

    while true
        % Attempt to retrieve the latest LaserScan message
        scanMsg = scanSub.LatestMessage;
        
        if ~isempty(scanMsg)
            % Extract ranges array
            ranges = scanMsg.ranges;

            % Convert NaN or Inf to a large value so they won't break min()
            validRanges = ranges;
            invalidMask = ~isfinite(validRanges);
            validRanges(invalidMask) = [];

            if ~isempty(validRanges)
                % If we have valid distance readings, find the minimum
                minDist = min(validRanges);
                fprintf("Nearest obstacle distance: %.3f m\n", minDist);
            else
                % If all are Inf or NaN
                disp("No valid obstacle data in this scan.");
            end
        else
            disp("No scan data received yet...");
        end
        
        pause(0.5);  % Check twice per second
    end
end
