clc; close all; clear all;

attacker_LatLon_subscriber = rossubscriber('/mavros/global_position/global');
attacker_angle_subscriber = rossubscriber('/mavros/global_position/compass_hdg');
attacker_velocity_publisher = rospublisher('/mavros/setpoint_velocity/cmd_vel_unstampped');

dt = 0.1;
vm = 0.4;
N = 10;

target=[];
target(1) = input('Target X: ');
target(2) = input('Target Y: ');

attacker_LatLon = receive(attacker_LatLon_subscriber,10);
attacker_angle = receive(attacker_angle_subscriber,10);

%% Real Vehicle Model
sys = ardurover_PN_RealVehicleROS(target, ...
                                  vm, ...
                                  attacker_LatLon_subscriber, ...
                                  attacker_angle_subscriber, ...
                                  attacker_velocity_publisher ...
                                  );
                      
if(attacker_angle.Data > 180)
    attacker_angle.Data = attacker_angle.Data - 2*180;
end
                              
sys.initialCondition = {[attacker_LatLon.Latitude;
                         attacker_LatLon.Longitude;
                         sqrt((target(1) - attacker_LatLon.Latitude)^2 + (target(2) - attacker_LatLon.Longitude)^2);
                         atan2(target(2) - attacker_LatLon.Longitude, target(1) - attacker_LatLon.Latitude);
                         3.14*attacker_angle.Data/180]};
                     
sys.controller = turtlesim_PN_IController(@(t,x) N*(-vm*sin(x(5)-x(4)))/x(3));

va = VirtualArena(sys,...
    'StoppingCriteria'  , @(t,x,sysList)sqrt((sys.x(1)-target(1))^2 + (sys.x(2)-target(2))^2)<=0.1,...
    'DiscretizationStep', dt ,...
    'RealTime'          , 1 ,...
    'PlottingStep'      , 1 );
  
log = va.run();