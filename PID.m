function [vel, integral, error_prev] = PID(position, goal, integral, error_prev, sampleTime)
    Kp = 5; 
    Ki = 0.2; 
    Kd = 0.1; 
    % distance
    dx = goal(1) - position(1);
    dy = goal(2) - position(2);
    theta_goal = atan2(dy, dx); % زاویه هدف
    theta_error = angleDifference(position(3), theta_goal); % اختلاف زاویه
    % PID control
    integral = integral + theta_error * sampleTime;
    derivative = (theta_error - error_prev) / sampleTime;
    error_prev = theta_error;
    % محاسبه سرعت
    v = 4; % سرعت خطی ثابت
    w = Kp * theta_error + Ki * integral + Kd * derivative; % سرعت زاویه‌ای
    vel = [v, w];

end

function delta = angleDifference(angle1, angle2)
    delta = mod(angle2 - angle1 + pi, 2*pi) - pi;
end
