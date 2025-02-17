%gazebo_matlab_node=ros2node("/gazebo");
%scanSub_gazebo=ros2subscriber(gazebo_matlab_node,"/scan");
syms y;
y=0;
for i = 1:128
    %disp(scanSub_gazebo.LatestMessage.ranges(i))
    if scanSub_gazebo.LatestMessage.ranges(i) < 0.1
        y=1;
    end
    
    end
    
end

disp(y);