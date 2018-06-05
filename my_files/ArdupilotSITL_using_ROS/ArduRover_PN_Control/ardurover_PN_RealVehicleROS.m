classdef ardurover_PN_RealVehicleROS < CtSystem

    properties
        target
        vm      % Attacker Velocity Magnitude
        attacker_LatLon_subscriber
        attacker_angle_subscriber
        attacker_velocity_publisher
    end
    
    methods
        
        function obj = ardurover_PN_RealVehicleROS(target, ...
                                                   vm, ...
                                                   attacker_LatLon_subscriber, ...
                                                   attacker_angle_subscriber, ...
                                                   attacker_velocity_publisher ...
                                                   )
                                      
            obj = obj@CtSystem('nx',5,'nu',1,'ny',5);
            obj.target = target;
            obj.vm = vm;
            obj.attacker_LatLon_subscriber = attacker_LatLon_subscriber;
            obj.attacker_angle_subscriber = attacker_angle_subscriber;
            obj.attacker_velocity_publisher = attacker_velocity_publisher;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            %% Publisher to send velocity to the vehicle;
            
            attacker_vel_Msg = rosmessage(obj.attacker_velocity_publisher);
            
            attacker_LatLon = receive(obj.attacker_LatLon_subscriber,10);
%             attacker_angle = receive(obj.attacker_angle_subscriber,10);
            
            distance = sqrt((attacker_LatLon.Latitude-obj.target(1))^2 + (attacker_LatLon.Longitude-obj.target(2))^2);
            
            if (distance >= 0.4)   
                attacker_vel_Msg.Linear.X = obj.vm;
                attacker_vel_Msg.Angular.Z = u(1);
            else
                attacker_vel_Msg.Linear.X = 0;
                attacker_vel_Msg.Angular.Z = 0;
            end    
            
            send(obj.attacker_velocity_publisher, attacker_vel_Msg);
            
            xDot = [obj.vm*cos(x(5));  %Velocity of Attacker in x-dircetion
                    obj.vm*sin(x(5));  %Velocity of Attacker in y-dircetion
                    -obj.vm*cos(x(4)-x(3));  %Velocity along Attacker-Target LOS
                    -obj.vm*sin(x(4)-x(3))/x(3);   %Angular velocity of above equation
                    u(1)    %Angular Velocity of Attacker
                    ];  
 
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber to read the position of the vehicle

            attacker_LatLon = receive(obj.attacker_position_subscriber,10);
            attacker_angle = receive(obj.attacker_angle_subscriber,10);
            
            if(attacker_angle.Data > 180)
                attacker_angle.Data = attacker_angle.Data - 2*180;
            end
            
            y = double([attacker_LatLon.Latitude;
                 attacker_LatLon.Longitude;
            	 sqrt((obj.target(1) - attacker_LatLon.Latitude)^2 + (obj.target(2) - attacker_LatLon.Longitude)^2);
                 atan2(obj.target(2) - attacker_LatLon.Longitude, obj.target(1) - attacker_LatLon.Latitude);
                 3.14*attacker_angle.Data/180]);
        
        end
    
    end
    
end