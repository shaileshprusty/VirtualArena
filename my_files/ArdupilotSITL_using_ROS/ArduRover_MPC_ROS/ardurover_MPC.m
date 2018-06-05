%PNPP
%moving target
%EKF
clc; close all; clear all;

attacker_LatLon_subscriber = rossubscriber('/attacker/mavros/global_position/global');
attacker_angle_subscriber = rossubscriber('/attacker/mavros/global_position/compass_hdg');
attacker_velocity_publisher = rospublisher('/attacker/mavros/setpoint_velocity/cmd_vel_unstampped');
target_LatLon_subscriber = rossubscriber('/target/mavros/global_position/global');
target_angle_subscriber = rossubscriber('/target/mavros/global_position/compass_hdg');
target_velocity_publisher = rospublisher('/target/mavros/setpoint_velocity/cmd_vel_unstampped');
defender_LatLon_subscriber = rossubscriber('/defender/mavros/global_position/global');
defender_angle_subscriber = rossubscriber('/defender/mavros/global_position/compass_hdg');
defender_velocity_publisher = rospublisher('/defender/mavros/setpoint_velocity/cmd_vel_unstampped');

dt = 1;
vm = 0.4;
vt = 0.2;
vd = 0.8;
K = 10;
N = 10;
switchGuidanceLaw = 10;

attacker_LatLon = receive(attacker_LatLon_subscriber,10);
attacker_angle = receive(attacker_angle_subscriber,10);
target_LatLon = receive(target_LatLon_subscriber,10);
target_angle = receive(target_angle_subscriber,10);
defender_LatLon = receive(defender_LatLon_subscriber,10);
defender_angle = receive(defender_angle_subscriber,10);

%% Real Vehicle Model
sys = turtlesim_MPC_RealVehicleROS(N, ...
                                   K, ...
                                   switchGuidanceLaw, ...
                                   vm, ...
                                   vt, ...
                                   vd, ...
                                   attacker_LatLon_subscriber, ...
                                   attacker_angle_subscriber, ...
                                   attacker_velocity_publisher, ...
                                   target_LatLon_subscriber, ...
                                   target_angle_subscriber, ...
                                   target_velocity_publisher, ...
                                   defender_LatLon_subscriber, ...
                                   defender_angle_subscriber, ...
                                   defender_velocity_publisher ...
                                   );

if(attacker_angle.Data > 180)
    attacker_angle.Data = attacker_angle.Data - 2*180;
end
                               
if(defender_angle.Data > 180)
    defender_angle.Data = defender_angle.Data - 2*180;
end
                               
sys.initialCondition = {double([attacker_LatLon.Latitude;
                                attacker_LatLon.Longitude;
                                defender_LatLon.Latitude;
                                defender_LatLon.Longitude;
                                3.14*defender_angle.Data/180;
                                target_LatLon.Latitude;
                                target_LatLon.Longitude;
                                sqrt((target_LatLon.Latitude - attacker_LatLon.Latitude)^2 + (target_LatLon.Longitude - attacker_LatLon.Longitude)^2);
                                atan2(target_LatLon.Longitude - attacker_LatLon.Longitude, target_LatLon.Latitude - attacker_LatLon.Latitude);
                                3.14*attacker_angle.Data/180])};

mpcOp = ICtMpcOp( ...
    'System'               , sys,...
    'HorizonLength'        , 2*dt,...
    'StageCost'            , @(t,x,u,varargin) -((x(6)-x(1))^2+(x(7)-x(2))^2)+((x(3)-x(1))^2+(x(4)-x(2))^2));

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtsys = DiscretizedSystem(sys,dt);

dtsys.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp, 'UseSymbolicEvaluation', 1) ...
    );

va = VirtualArena(dtsys,...
    'StoppingCriteria'  , @(t,sysList)sqrt(((dtsys.x(3)-dtsys.x(1))^2+(dtsys.x(4)-dtsys.x(2))^2))<0.5 || sqrt(((dtsys.x(6)-dtsys.x(1))^2+(dtsys.x(7)-dtsys.x(2))^2))<0.5,...
    'DiscretizationStep', dt ,...
    'PlottingStep'      , 1/0.1,...
    'RealTime'          , 1 ...
    );

log = va.run();