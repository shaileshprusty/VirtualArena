classdef ardurover_RealVehicleROS_utm < CtSystem

    properties
%         flag=0
        target
        velocity_magnitude
        LatLon_subscriber
        angle_subscriber
        velocity_publisher
    end
    
    methods
        
        function obj = ardurover_RealVehicleROS_utm(LatLon_subscriber, angle_subscriber, velocity_publisher, target, velocity_magnitude)
            obj = obj@CtSystem('nx',3,'nu',1,'ny',3);
            obj.LatLon_subscriber = LatLon_subscriber;
            obj.angle_subscriber = angle_subscriber;
            obj.target = target;
            obj.velocity_magnitude = velocity_magnitude;
            obj.velocity_publisher = velocity_publisher;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            %% Publisher to send velocity to the vehicle;
            
            vel_Msg = rosmessage(obj.velocity_publisher);
            rover_LatLon = receive(obj.LatLon_subscriber,10);
%             rover_angle = receive(obj.angle_subscriber,10);
            
            [utmX, utmY] = deg2utm(rover_LatLon.Latitude, rover_LatLon.Longitude);
            
            distance = sqrt((utmX-obj.target(1))^2 + (utmY-obj.target(2))^2)
            
%             if (distance >= 0.2 && obj.flag == 1)
            if (distance >= 0.2)   
                vel_Msg.Linear.X = obj.velocity_magnitude;
                vel_Msg.Angular.Z = u(1);
                send(obj.velocity_publisher,vel_Msg);
%                 obj.flag=0;
                disp(u(1));
            else
                vel_Msg.Linear.X = 0;
                vel_Msg.Angular.Z = 0;
                send(obj.velocity_publisher,vel_Msg);
                disp(0);
            end    

            xDot = [obj.velocity_magnitude*cos(x(3));obj.velocity_magnitude*sin(x(3));u(1)];

        
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber to read the position of the vehicle

            rover_LatLon = receive(obj.LatLon_subscriber,10);
            rover_angle = receive(obj.angle_subscriber,10);

            [utmX, utmY] = deg2utm(rover_LatLon.Latitude, rover_LatLon.Longitude);
            
%             obj.flag = 1;
            
            y = [utmX;
                utmY;
                deg2rad(90 - rover_angle.Data)];
        
        end
    
    end
    
end