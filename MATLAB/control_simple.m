%% controlTurtlebot3.m


setenv('ROS_DOMAIN_ID','30');
% 1. Create a ROS 2 node. 
turtle_node = ros2node("/turtlebot3_node");
% 2. Create a subscriber to /cmd_vel
cmdSub = ros2subscriber(turtle_node, "/cmd_vel", "geometry_msgs/Twist");
% 3. Create a publisher to /cmd_vel 
cmdPub = ros2publisher(turtle_node, "/cmd_vel", "geometry_msgs/Twist");
% 4. Create a Twist message from the publisher's type.
cmdMsg = ros2message(cmdPub);
% 5. Set the linear and angular velocity fields.
%geometry_msgs/Twist has:
%cmdMsg.linear.x, cmdMsg.linear.y, cmdMsg.linear.z
%cmdMsg.angular.x, cmdMsg.angular.y, cmdMsg.angular.z
cmdMsg.linear.x =0.0;
cmdMsg.linear.y=0.0;
cmdMsg.linear.z=0.0;
cmdMsg.angular.x=0.0;
cmdMsg.angular.y=0.0;
cmdMsg.angular.z=0.0;
% 6. Send the velocity command to the TurtleBot3.
send(cmdPub,cmdMsg);
disp("Velocity command sent to TurtleBot3!");