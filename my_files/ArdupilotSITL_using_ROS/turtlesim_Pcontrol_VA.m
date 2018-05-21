clc; close all;
mysub = rossubscriber('/turtle1/pose');
%[mypub , pubmsg] = rospublisher('/turtle1/cmd_vel');

dt = 1;
kp =1.5;
V=0.8;

recvMsg = mysub.LatestMessage;

a = input('Target X: ');
b = input('Target Y: ');

sys = ICtSystem(...
    'StateEquation', @(t,x,u,varargin) [
    V*cos(x(3));
    V*sin(x(3));
    u(1)],...
    'nx',3,'nu',1 ...
    );

sys.controller = IController(@(t,x)(kp*(atan2((b - x(2)),(a - x(1))) - x(3))));

sys.initialCondition = {[recvMsg.X;recvMsg.Y;recvMsg.Theta]};

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-a)*(sys.x(1)-a) + (sys.x(2)-b)*(sys.x(2)-b))<=0.1,...
    'DiscretizationStep', dt ,...
    'PlottingStep'      , 1  ,...
    'StepPlotFunction'  , @ex01StepPlotFunction);

log = va.run();