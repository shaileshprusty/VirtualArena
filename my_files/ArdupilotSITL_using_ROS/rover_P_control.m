clc; clear all; close all; 

LatLon_sub = rossubscriber('/mavros/global_position/global');
angle_sub = rossubscriber('/mavros/global_position/compass_hdg');
[velocity_pub , vel_msg] = rospublisher('/mavros/setpoint_velocity/cmd_vel_unstamped');

% target_LA = input('Target LA : ');
% target_LO = input('Target LO : ');

target_LA = 28.5448836;
target_LO = 77.2712178;

[utm_targetX, utm_targetY] = deg2utm(target_LA,target_LO);

Vk = 0.3;
kp = 0.01;

while(1)
    [utmX,utmY] = deg2utm(LatLon_sub.LatestMessage.Latitude, LatLon_sub.LatestMessage.Longitude);

    theta = 90 - angle_sub.LatestMessage.Data;

    LOS_dist = sqrt((utm_targetY-utmY)^2+(utm_targetX-utmX)^2);

    alpha = atan2((utm_targetY-utmY),(utm_targetX-utmX));

    angle_error = rem(rad2deg(alpha) - theta,360);
    
    if (LOS_dist >= 0.2)
%         vel_msg.Linear.X = Vk*(1 - abs(angle_error)/180);
        vel_msg.Linear.X = Vk;
        vel_msg.Angular.Z = kp*angle_error;
        send(velocity_pub , vel_msg);
    else
        vel_msg.Linear.X = 0;
        vel_msg.Linear.Y = 0;
        vel_msg.Angular.Z = 0;
        send(velocity_pub , vel_msg);
        break;
    end
    pause(0.656)
end