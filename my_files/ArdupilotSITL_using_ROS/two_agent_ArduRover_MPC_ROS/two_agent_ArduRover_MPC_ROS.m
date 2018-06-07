%% Using MPC controller to decrease the distance between two turtle.

% Turtle1 moving with constant velocity
% Turtle2 using control variable from MPC to minimize the given cost function.

clc; close all; clear all;

% publisher,subscriber and other required initializations....

attacker_LatLon_subscriber = rossubscriber('/attacker/mavros/global_position/global');
attacker_angle_subscriber = rossubscriber('/attacker/mavros/global_position/compass_hdg');
attacker_velocity_publisher = rospublisher('/attacker/mavros/setpoint_velocity/cmd_vel_unstamped');

target_LatLon_subscriber = rossubscriber('/target/mavros/global_position/global');
target_angle_subscriber = rossubscriber('/target/mavros/global_position/compass_hdg');
target_velocity_publisher = rospublisher('/target/mavros/setpoint_velocity/cmd_vel_unstamped');

target_LatLon = receive(target_LatLon_subscriber , 10);
attacker_LatLon = receive(attacker_LatLon_subscriber , 10);
attacker_angle = receive(attacker_angle_subscriber , 10);

dt = 1;
vm = 80;
vt = 20;

%system initialization.
realSystem = two_agent_ArduRover_MPC_RealVehicleROS(target_LatLon_subscriber,...
                                                    target_angle_subscriber,...
                                                    target_velocity_publisher,...
                                                    attacker_LatLon_subscriber,...
                                                    attacker_angle_subscriber,...
                                                    attacker_velocity_publisher,...
                                                    vm,...
                                                    vt);    

%% Angle correction for turtlesim in order to constain it between -pi to pi 

if( attacker_angle.Data > 180 )
   attacker_angle.Data = attacker_angle.Data - 2*180;
end

%% ........

realSystem.initialCondition = {[attacker_LatLon.Latitude;
                                       attacker_LatLon.Longitude;
                                       3.14*attacker_angle.Data/180;
                                       target_LatLon.Latitude;
                                       target_LatLon.Longitude;
                                       sqrt((target_LatLon.Latitude - attacker_LatLon.Latitude)^2 + (target_LatLon.Longitude - attacker_LatLon.Longitude)^2);
                                       (atan2((target_LatLon.Longitude - attacker_LatLon.Longitude),(target_LatLon.Latitude - attacker_LatLon.Latitude)))]};
                                                    
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
                  'StoppingCriteria'    , @(t,sysList)sqrt(((dtRealSystem.x(4)-dtRealSystem.x(1))^2+(dtRealSystem.x(5)-dtRealSystem.x(2))^2))<0.00004 ,...
                  'DiscretizationStep'  , dt ,...
                  'RealTime'            , 1/dt , ...
                  'PlottingStep'        , 1/dt ...
                 );
             
log = va.run();