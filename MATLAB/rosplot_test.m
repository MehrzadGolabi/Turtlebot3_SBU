setenv('ROS_DOMAIN_ID','30');
scan_node = ros2node("/plot_lidar");
ScanSub = ros2subscriber(scan_node,"/scan","sensor_msgs/LaserScan");
scanMsg = receive(ScanSub);

rosPlot(scanMsg);