clc;clear;close all;

% load map
load ('C:\Users\Amirarjmandi\Desktop\new task\naghshe.mat')
map1 = binaryOccupancyMap(simpleMap,1);
[mapdimx, mapdimy] = size(simpleMap);
map2 = binaryOccupancyMap(mapdimy, mapdimx, 10);

figure(1); %3 in 1 plot
subplot(1,3,3);
    show(map1);
    title(' ');

%% controler and sensor
diffDrive = differentialDriveKinematics("VehicleInputs", "VehicleSpeedHeadingRate");
lidar = rangeSensor;
lidar.Range = [0,10];

%% path planing
path = [5 3;4 21;15 23];
initPose = [path(1,1) path(1,2), 0];
goal = [path(2,1) path(2,2)]';
goal1 = [path(3,1) path(3,2)]';

%% parametr
sampleTime = 0.05;
t = 0:sampleTime:100;
poses = zeros(3, numel(t));
estPose = zeros(3, numel(t));
Idealposes = zeros(3, numel(t));
difrence = zeros(2,numel(t));
poses(:,1) = initPose'; 
Idealposes(:,1) = initPose';
estPose(:,1) = initPose'; 
countif = 0;

%% mont carlo parametr
numParticles = 220;
particles = repmat(initPose', 1, numParticles) + 0.2 * randn(3, numParticles);
weights = ones(1, numParticles) / numParticles;

%% pid parametr

integral0 = 0;   % Integral of error
integral1 = 0;
error_prev0 = 0;
error_prev1 = 0;
%% farayand
for idx = 1:numel(t)
    dist = norm(goal - Idealposes(1:2,idx));
    if (dist < 0.1 && countif == 0)
        goal = goal1;
        countif = 1;
    elseif (dist < 0.1 && countif == 1)
        break;
    end
    
    % update naghshe
    [ranges, angles] = lidar(poses(:,idx)', map1);
    scan = lidarScan(ranges, angles);
    validScan = removeInvalidData(scan, 'RangeLimits', [0, 10]);
    insertRay(map2, poses(:,idx)', validScan, 10);
        
    % control
    [vel0, integral0, error_prev0] = PID(estPose(:,idx),goal,integral0,error_prev0,sampleTime); %robot talash mikonad refrence ra follow konad 
    [vel01, integral1, error_prev1] = PID(Idealposes(:,idx),goal,integral1,error_prev1,sampleTime);
    move = derivative(diffDrive, poses(:,idx), vel0);
    move1 = derivative(diffDrive, Idealposes(:,idx), vel01);
    
    poses(:,idx+1) = poses(:,idx) + move * sampleTime;
    Idealposes(:,idx+1) = Idealposes(:,idx) + move1 * sampleTime;

    difrence(1,idx)= idx;
    difrence(2,idx)= norm(Idealposes(:,idx)'-poses(:,idx)');

    % mont carlo localization
    [estPose(:,idx+1), particles]= MCL(particles,numParticles,map1,validScan,move,sampleTime,lidar);

    %figure
    figures(path,poses,Idealposes,estPose,map1,map2,particles,idx,difrence)  

end

%% functions

