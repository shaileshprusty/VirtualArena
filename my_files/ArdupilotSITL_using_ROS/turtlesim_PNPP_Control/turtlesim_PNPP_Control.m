clc; close all; clear all;

attacker_position_subscriber = rossubscriber('/turtle2/pose');
attacker_velocity_publisher = rospublisher('/turtle2/cmd_vel');
target_position_subscriber = rossubscriber('/turtle1/pose');
target_velocity_publisher = rospublisher('/turtle1/cmd_vel');

dt = 0.1;
vm = 0.4;
vt = 0.2;
K = 3;
N = 3;
switchGuidanceLaw = 10;

attacker_position = receive(attacker_position_subscriber,10);
target_position = receive(target_position_subscriber,10);

%% Real Vehicle Model
sys = turtlesim_PNPP_RealVehicleROS(N, ...
                                   K, ...
                                   switchGuidanceLaw, ...
                                   vm, ...
                                   vt, ...
                                   attacker_position_subscriber, ...
                                   attacker_velocity_publisher, ...
                                   target_position_subscriber, ...
                                   target_velocity_publisher ...
                                  );
                             
sys.initialCondition = {[attacker_position.X;
                         attacker_position.Y;
%                          defender_position.X;
%                          defender_position.Y;
%                          defender_position.Theta;
                         target_position.X;
                         target_position.Y;
                         sqrt((target_position.X - attacker_position.X)^2 + (target_position.Y - attacker_position.Y)^2);
                         atan2(target_position.Y - attacker_position.Y, target_position.X - attacker_position.X);
                         attacker_position.Theta]};
                     
sys.controller = turtlesim_IController(@(t,x) x(7));

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-sys.x(3))^2 + (sys.x(2)-sys.x(2))^2)<=0.1,...
    'DiscretizationStep', dt ,...
    'RealTime'          , 1 ,...
    'PlottingStep'      , 1 ,...
    'StepPlotFunction'  , @ex01StepPlotFunction);
  
log = va.run();