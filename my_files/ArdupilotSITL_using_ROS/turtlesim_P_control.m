
mysub = rossubscriber('/turtle1/pose');
[mypub , pubmsg] = rospublisher('/turtle1/cmd_vel');
X = 10;
Y = 10;
k = 1.5;

while(1)
recvMsg = mysub.LatestMessage;
x = recvMsg.X;
y = recvMsg.Y;
theta = recvMsg.Theta;
d=sqrt((X-x)*(X-x) + (Y-y)*(Y-y));
    phi = atan2((Y-y),(X-x));
    beta = phi - theta;
    if (beta >= 3.14)
          beta = beta - 2*3.14;
          
    end
    if (beta <= -3.14)
          beta = 2*3.14 + beta;
    end 
        
    if(d <= 0.2)
	
		pubmsg.Linear.X=0;
		pubmsg.Angular.Z=0;    

	else
    	
		pubmsg.Linear.X=1;
		pubmsg.Angular.Z=k*beta; 
    end

send(mypub , pubmsg);

end