% AR PA1
% Jackie Scanlon
% This code is to show that the area under the linear velocity curve matches the 
% position of the turtle. Thus, turtlesim calculates position correctly.

% Get the rosbag
bag = rosbag('recordedbag.bag');

% Get the pose topic and extra the data into vectors
bag_pose = select(bag, 'Topic', '/turtle1/pose');

% Get the position data
ts = timeseries(bag_pose, 'X');

% Save the time vector
time = ts.Time;

% Offset the time vector so that time starts at 0
t0 = time(1);
time = time - t0;

% Get the interval of spacing
dt = time(2);

% Number of points to plot
n = size(time,1);

% Put the position data into a vector
pos = ts.Data;

% Put the theta data into a vector
theta = timeseries(bag_pose, 'Theta').Data;

% Put the linear velocity (in the turtle frame) into a vector.
% X velocity is as reported by the LinearVelocity portion of the
% pose message, and the Y velocity is 0 (in this frame).
vel = [timeseries(bag_pose, 'LinearVelocity').Data zeros(n,1)];

% Get the start position so we can offset our area under the curve
% by it later
start = pos(1);

% Convert vel in the turtle frame into vel in world frame
% Velocity in world frame is called vel_w
vel_w = zeros(n, 2);

% Area under the curve variable - AUC
AUC = zeros(n,1);

% For each point
for i=1:n
    
    % Rotation matrix
    R = [cos(theta(i)) -sin(theta(i));
     sin(theta(i)) cos(theta(i))];
 
    % Convert vel in turtle frame to vel in world frame
    vel_w(i,:) = (R*vel(i,:)')';
    
    % Calculate area under the curve up to this point
    % and offset by the starting position ('+C' in integral)
    AUC(i) = dt*trapz(vel_w(1:i,1)) + start;
end

% Plot our results
figure(1)
plot(time, vel_w(:,1), time, vel(:,1));
title('Linear x-velocity in turtle frame and in world frame');
xlabel('Time [sec]');
ylabel('Velocity [units/sec]');
legend('x-velocity in world frame', 'x-velocity in turtle frame');
ylim([-1 1.5]);

figure(2)
plot(time, pos, time, AUC);
title('Reported position and calculated position');
xlabel('Time [sec]');
ylabel('Position [units]');
legend('Position reported by turtlesim', 'Position calculated from velocity');


