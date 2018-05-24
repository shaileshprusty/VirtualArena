clc; close all;clear all;


mysub1 = rossubscriber('/mavros/global_position/global');
mysub2 = rossubscriber('/mavros/global_position/compass_hdg');
[mypub , pubmsg] = rospublisher('/mavros/setpoint_velocity/cmd_vel_unstamped');

position_Msg = receive(mysub1 , 10);
angle_Msg = receive(mysub2 , 10);

dt = 1;
kp = 1;
Velocity = 500;

a = input('Target X: ');
b = input('Target Y: ');

sys = ICtSystem(...
    'StateEquation', @(t,x,u,varargin) [
    Velocity*cos(x(3));
    Velocity*sin(x(3));
    u(1)],...
    'nx',3,'nu',1 ...
    );

sys.controller = IController(@(t,x)(kp*(3.14*x(3)/180 - atan2((b - x(2)),(a - x(1))))));

sys.initialCondition = {[position_Msg.Latitude;position_Msg.Longitude;3.14*angle_Msg.Data/180]};

va = ArduRover_VirtualArena(sys,...
    'Target', [a;b],...
    'Velocity', Velocity,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-a)*(sys.x(1)-a) + (sys.x(2)-b)*(sys.x(2)-b))<=0.0002,...
    'DiscretizationStep', dt ,...
    'PlottingStep'      , 1 );
    %'StepPlotFunction'  , @ex01StepPlotFunction);

log = va.run();