%% Using MPC controller to decrease the distance between turtle and target.

% Turtle1 using control variable from MPC to minimize the given cost function.

clc; close all; clear all;

% publisher,subscriber and other required initializations....

position_subscriber = rossubscriber('/turtle1/pose');
velocity_publisher = rospublisher('/turtle1/cmd_vel');

position_Msg = receive(position_subscriber, 10);

dt = 1;
velocity_magnitude = 1;

a = input('Target X: ');
b = input('Target Y: ');

%system initialization.
realSystem = single_agent_turtlesim_MPC_RealVehicleROS(position_subscriber, ...
                                                       velocity_publisher, ...
                                                       [a;b], ...
                                                       velocity_magnitude ...
                                                       );    

%% Angle correction for turtlesim in order to constain it between -pi to pi 

if( position_Msg.Theta > 3.14 )
   position_Msg.Theta = position_Msg.Theta - 2*3.14;
end

if( position_Msg.Theta < -3.14 )
   position_Msg.Theta = position_Msg.Theta + 2*3.14;
end

%% ........

realSystem.initialCondition = {double([position_Msg.X;
                                       position_Msg.Y;
                                       position_Msg.Theta;
                                       sqrt((a - position_Msg.X)^2 + (b - position_Msg.Y)^2);
                                       (atan2((b - position_Msg.Y),(a - position_Msg.X)))])};
                                                    
mpcOp = ICtMpcOp( ...
                'System'               , realSystem,...
                'HorizonLength'        , 2*dt,...
                'StageCost'            , @(t,x,u,varargin) -x(4));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(realSystem,dt);

dtRealSystem.controller = MpcController(...
                                      'MpcOp'       , dtMpcOp ,...
                                      'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',1) ...
                                       );

%% VirtualArena Object defined.                                                                      
va = VirtualArena(dtRealSystem,...
                  'StoppingCriteria'  , @(t,sysList)sqrt(((a-dtRealSystem.x(1))^2+(b-dtRealSystem.x(2))^2))<=0.1 ,...
                  'DiscretizationStep', dt ,...
                  'RealTime'          , 1/dt , ...
                  'PlottingStep'      , 1/dt ...
                 );
             
log = va.run();