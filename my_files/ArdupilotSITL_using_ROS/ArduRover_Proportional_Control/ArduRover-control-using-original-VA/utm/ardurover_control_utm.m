clc; clear all; close all;

LatLon_subscriber = rossubscriber('/mavros/global_position/global');
angle_subscriber = rossubscriber('/mavros/global_position/compass_hdg');
velocity_publisher = rospublisher('/mavros/setpoint_velocity/cmd_vel_unstamped');

velocity_magnitude = 0.3;
dt = 1;
kp = 0.01; 

LatLon_Msg = receive(LatLon_subscriber, 10);
angle_Msg = receive(angle_subscriber, 10);

% a = input('Target X: ');
% b = input('Target Y: ');

a = 28.5448836;
b = 77.2712178;

[a,b] = deg2utm(a,b);

sys = ardurover_RealVehicleROS_utm(LatLon_subscriber, angle_subscriber, velocity_publisher, [a;b], velocity_magnitude);

[utmX, utmY] = deg2utm(LatLon_Msg.Latitude, LatLon_Msg.Longitude);

sys.initialCondition = {[utmX; utmY; deg2rad(90 - angle_Msg.Data)]};

sys.controller = ardurover_IController_utm(@(t,x) rad2deg(atan2(b - x(2), a - x(1)) - x(3)), kp);

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-a)^2 + (sys.x(2)-b)^2)<=0.2,...
    'DiscretizationStep', dt ,...
    'RealTime'          , 1);
%     'PlottingStep'      , 1 ,...
%     'StepPlotFunction'  , @ex01StepPlotFunction);
  
log = va.run();