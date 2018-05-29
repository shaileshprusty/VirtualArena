clc; clear all; close all;

position_subscriber = rossubscriber('/turtle1/pose');
velocity_publisher = rospublisher('/turtle1/cmd_vel');

velocity_magnitude = 1;
dt = 0.1;
kp = 1; 

position_Msg = receive(position_subscriber, 10);

a = input('Target X: ');
b = input('Target Y: ');

sys = turtlesim_RealVehicleROS(position_subscriber, velocity_publisher, [a;b], velocity_magnitude);

sys.initialCondition = {[position_Msg.X;position_Msg.Y;position_Msg.Theta]};

sys.controller = turtlesim_IController(@(t,x) atan2(b - x(2), a - x(1)) - x(3), kp);

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-a)*(sys.x(1)-a) + (sys.x(2)-b)*(sys.x(2)-b))<=0.1,...
    'DiscretizationStep', dt ,...
    'RealTime'          , 1 ,...
    'PlottingStep'      , 1 ,...
    'StepPlotFunction'  , @ex01StepPlotFunction);
  
log = va.run();