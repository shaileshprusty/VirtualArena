clc; close all; clear all;

attacker_position_subscriber = rossubscriber('/turtle1/pose');
attacker_velocity_publisher = rospublisher('/turtle1/cmd_vel');

dt = 0.1;
vm = 0.4;
K = 3;
N = 3;
switchGuidanceLaw = 10;

target=[];
target(1) = input('Target X: ');
target(2) = input('Target Y: ');

attacker_position = receive(attacker_position_subscriber,10);

%% Real Vehicle Model
sys = turtlesim_PNPP_RealVehicleROS(target, ...
                                    N, ...
                                    K, ...
                                    switchGuidanceLaw, ...
                                    vm, ...
                                    attacker_position_subscriber, ...
                                    attacker_velocity_publisher ...
                                    );
                             
sys.initialCondition = {[attacker_position.X;
                         attacker_position.Y;
                         sqrt((target(1) - attacker_position.X)^2 + (target(2) - attacker_position.Y)^2);
                         atan2(target(2) - attacker_position.Y, target(1) - attacker_position.X);
                         attacker_position.Theta]};
                     
sys.controller = turtlesim_PNPP_IController(@(t,x) ((((-1).^floor(ceil(t/switchGuidanceLaw)))+1)/2)*(N*(-vm*sin(x(5)-x(4)))/x(3)))+(1-((((-1).^floor(ceil(t/switchGuidanceLaw)))+1)/2))*(((K*(x(5)-x(4)))/vm));

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-sys.x(3))^2 + (sys.x(2)-sys.x(2))^2)<=0.1,...
    'DiscretizationStep', dt ,...
    'RealTime'          , 1 ,...
    'PlottingStep'      , 1 ,...
    'StepPlotFunction'  , @ex01StepPlotFunction);
  
log = va.run();