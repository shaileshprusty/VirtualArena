classdef turtlesim_RealVehicleROS < CtSystem

    properties
        target
        velocity_magnitude
        position_subscriber 
        velocity_publisher
    end
    
    methods
        
        function obj = turtlesim_RealVehicleROS(position_subscriber, velocity_publisher, target, velocity_magnitude)
            obj = obj@CtSystem('nx',3,'nu',1,'ny',3);
            obj.position_subscriber = position_subscriber;
            obj.target = target;
            obj.velocity_magnitude = velocity_magnitude;
            obj.velocity_publisher = velocity_publisher;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            %% Publisher to send velocity to the vehicle;
            
            vel_Msg = rosmessage(obj.velocity_publisher);
            turtle_position = receive(obj.position_subscriber,10);
            position_params = [turtle_position.X;turtle_position.Y;turtle_position.Theta];  %Convert msg variable to matrix
            distance = sqrt((position_params(1)-obj.target(1))*(position_params(1)-obj.target(1)) + (position_params(2)-obj.target(2))*(position_params(2)-obj.target(2)));
            
            if (distance >= 0.1)   
                vel_Msg.Linear.X = obj.velocity_magnitude;
                vel_Msg.Angular.Z = u(1);
                send(obj.velocity_publisher,vel_Msg);
            else
                vel_Msg.Linear.X = 0;
                vel_Msg.Angular.Z = 0;
                send(obj.velocity_publisher,vel_Msg);
            end    

            xDot = [1*cos(x(3));1*sin(x(3));u(1)];

        
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber to read position of the vehicle

            turtle_position = receive(obj.position_subscriber,10) ;
            y = [turtle_position.X;
                 turtle_position.Y;
            	 turtle_position.Theta];
        
        end
    
    end
    
end