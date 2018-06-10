 clc; clear all; close all;

LatLon_subscriber = rossubscriber('/mavros/global_position/global');
angle_subscriber = rossubscriber('/mavros/global_position/compass_hdg');
velocity_publisher = rospublisher('/mavros/setpoint_velocity/cmd_vel_unstamped');

velocity_magnitude = 500;
dt = 0.1;
kp = 1; 

LatLon_Msg = receive(LatLon_subscriber, 10);
angle_Msg = receive(angle_subscriber, 10);

a = input('Target X: ');
b = input('Target Y: ');

sys = ardurover_RealVehicleROS(LatLon_subscriber, angle_subscriber, velocity_publisher, [a;b], velocity_magnitude);

sys.initialCondition = {[LatLon_Msg.Latitude;LatLon_Msg.Longitude;angle_Msg.Data]};

sys.controller = ardurover_IController(@(t,x) 3.14*x(3)/180 - atan2(b - x(2), a - x(1)), kp);

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-a)*(sys.x(1)-a) + (sys.x(2)-b)*(sys.x(2)-b))<=0.0002,...
    'DiscretizationStep', dt ,...
    'RealTime'          , 1);
%     'PlottingStep'      , 1 ,...
%     'StepPlotFunction'  , @ex01StepPlotFunction);
  
log = va.run();