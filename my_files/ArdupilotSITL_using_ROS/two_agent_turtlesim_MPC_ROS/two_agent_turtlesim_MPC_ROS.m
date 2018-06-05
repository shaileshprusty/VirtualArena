%% Using MPC controller to decrease the distance between two turtle.

% Turtle1 moving with constant velocity
% Turtle2 using control variable from MPC to minimize the given cost function.

clc; close all; clear all;

% publisher,subscriber and other required initializations....

target_position_subscriber = rossubscriber('/turtle1/pose');
target_velocity_publisher = rospublisher('/turtle1/cmd_vel');

attacker_position_subscriber = rossubscriber('/turtle2/pose');
attacker_velocity_publisher = rospublisher('/turtle2/cmd_vel');

target_position = receive(target_position_subscriber , 10);
attacker_position = receive(attacker_position_subscriber , 10);

dt = 1;
vm = 0.8;
vt = 0.2;

%system initialization.
realSystem = two_agent_turtlesim_MPC_RealVehicleROS(target_position_subscriber,...
                                      target_velocity_publisher,...
                                      attacker_position_subscriber,...
                                      attacker_velocity_publisher,...
                                      vm,...
                                      vt);    

%% Angle correction for turtlesim in order to constain it between -pi to pi 

if( attacker_position.Theta > 3.14 )
   attacker_position.Theta = attacker_position.Theta - 2*3.14;
end

if( attacker_position.Theta < -3.14 )
   attacker_position.Theta = attacker_position.Theta + 2*3.14;
end

%% ........

realSystem.initialCondition = {double([attacker_position.X;
                                       attacker_position.Y;
                                       attacker_position.Theta;
                                       target_position.X;
                                       target_position.Y;
                                       sqrt((target_position.X - attacker_position.X)^2 + (target_position.Y - attacker_position.Y)^2);
                                       (atan2((target_position.Y - attacker_position.Y),(target_position.X - attacker_position.X)))])};
                                                    
mpcOp = ICtMpcOp( ...
                'System'               , realSystem,...
                'HorizonLength'        , 2*dt,...
                'StageCost'            , @(t,x,u,varargin) -x(6));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
                                      'MpcOp'       , dtMpcOp ,...
                                      'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
                                       );

%% VirtualArena Object defined.                                                                      
va = VirtualArena(dtRealSystem,...
                  'StoppingCriteria'  , @(t,sysList)sqrt(((dtRealSystem.x(4)-dtRealSystem.x(1))^2+(dtRealSystem.x(5)-dtRealSystem.x(2))^2))<0.4 ,...
                  'DiscretizationStep', dt ,...
                  'RealTime',1/dt , ...
                  'PlottingStep'      , 1/dt ...
                 );
             
log = va.run();