clc; clear; close all;
 
mysub = rossubscriber('/turtle1/pose');
 
position_msg = receive(mysub,10);

dt = 1;
kp =1.5;
Velocity=0.8;

a = input('Target X: ');
b = input('Target Y: ');

sys = ICtSystem(...
    'StateEquation', @(t,x,u,varargin) [
    Velocity*cos(x(3));
    Velocity*sin(x(3));
    u(1)],...
    'nx',3,'nu',1 ...
    );

sys.initialCondition = {[position_msg.X;position_msg.Y;position_msg.Theta]};

sys.controller = IController(@(t,x)(kp*(atan2((b - x(2)),(a - x(1))) - x(3))));

va = turtlesim_VirtualArena(sys,...
    'Velocity'          , Velocity,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-a)*(sys.x(1)-a) + (sys.x(2)-b)*(sys.x(2)-b))<=0.1,...
    'DiscretizationStep', dt ,...
    'PlottingStep'      , 1  );
    %'StepPlotFunction'  , @ex01StepPlotFunction);

log = va.run();