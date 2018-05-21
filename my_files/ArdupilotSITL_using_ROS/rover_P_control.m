mysub1 = rossubscriber('/mavros/global_position/global');
mysub2 = rossubscriber('/mavros/global_position/compass_hdg');
[mypub , pubmsg] = rospublisher('/mavros/setpoint_velocity/cmd_vel_unstamped');

target_LO = input('Target LO : ');
target_LA = input('Target LA : ');

Vk = 500;
kp = 0.5;

while(1)
position_LA = mysub1.LatestMessage.Latitude;
position_LO = mysub1.LatestMessage.Longitude;

theta = mysub2.LatestMessage.Data;

LOS_dist = sqrt((target_LO-position_LO)*(target_LO-position_LO)+(target_LA-position_LA)*(target_LA-position_LA));

alpha = atan2((target_LO-position_LO),(target_LA-position_LA));

beta = 3.14*theta/180.0 - alpha;
  if (beta >= 3.14)
    beta = beta - 2*3.14;
  end
  if (beta <= -3.14)
    beta = beta + 2*3.14;
  end
  if (LOS_dist >= 0.00012)   
    pubmsg.Linear.X = Vk;
    pubmsg.Angular.Z = kp*beta;
  else
    pubmsg.Linear.X = 0;
    pubmsg.Linear.Y = 0;
    pubmsg.Angular.Z = 0;
  end
  
send(mypub , pubmsg);
end