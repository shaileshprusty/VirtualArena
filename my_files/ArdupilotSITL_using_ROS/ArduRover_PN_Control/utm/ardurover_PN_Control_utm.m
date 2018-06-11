clc; clear all; close all;

LatLon_subscriber = rossubscriber('/mavros/global_position/global');
angle_subscriber = rossubscriber('/mavros/global_position/compass_hdg');
velocity_publisher = rospublisher('/mavros/setpoint_velocity/cmd_vel_unstamped');

velocity_magnitude = 0.3;
dt = 1;
N = 0.01; 

LatLon_Msg = receive(LatLon_subscriber, 10);
angle_Msg = receive(angle_subscriber, 10);

% if(angle_Msg.Data < 270)
%     angle_Msg.Data = 90 - angle_Msg.Data;
% else
%     angle_Msg.Data = 90 - angle_Msg.Data + 360;
% end

a = input('Target X: ');
b = input('Target Y: ');

[a,b] = deg2utm(a,b);

sys = ardurover_PN_RealVehicleROS_utm(LatLon_subscriber, angle_subscriber, velocity_publisher, [a;b], velocity_magnitude);

[utmX, utmY] = deg2utm(LatLon_Msg.Latitude, LatLon_Msg.Longitude);

sys.initialCondition = {[utmX; utmY; deg2rad(90 - angle_Msg.Data)]};

sys.controller = ardurover_PN_IController_utm(@(t,x) rad2deg((-vm*sin(x(5)-x(4)))/x(3)), N);

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-a)^2 + (sys.x(2)-b)^2)<=2,...
    'DiscretizationStep', dt ,...
    'RealTime'          , 1);
%     'PlottingStep'      , 1 ,...
%     'StepPlotFunction'  , @ex01StepPlotFunction);
  
log = va.run();