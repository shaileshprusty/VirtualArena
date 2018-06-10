classdef single_agent_turtlesim_MPC_RealVehicleROS < CtSystem

    properties
         
        count = 0
        position_subscriber
        velocity_publisher
        target
        velocity_magnitude
    end
    
    methods
        
        function obj = single_agent_turtlesim_MPC_RealVehicleROS(position_subscriber, ...
                                                                 velocity_publisher, ...
                                                                 target, ...
                                                                 velocity_magnitude ...
                                                                 )
            
            obj = obj@CtSystem('nx',5,'nu',1,'ny',5);            
            obj.position_subscriber = position_subscriber;
            obj.velocity_publisher = velocity_publisher;
            obj.target = target;
            obj.velocity_magnitude = velocity_magnitude;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            % Publisher send u to the vehicle;
            vel_Msg = rosmessage(obj.velocity_publisher);
            turtle_position = receive(obj.position_subscriber,10);
            position_params = [turtle_position.X;turtle_position.Y;turtle_position.Theta];  %Convert msg variable to matrix
            distance = sqrt((position_params(1)-obj.target(1))*(position_params(1)-obj.target(1)) + (position_params(2)-obj.target(2))*(position_params(2)-obj.target(2)));
            
            if obj.count > 2             %to avoid initial random values.
                disp('--------publishing--------');
                if (distance >= 0.1)   
                    vel_Msg.Linear.X = obj.velocity_magnitude;
                    vel_Msg.Angular.Z = u(1);
                    send(obj.velocity_publisher,vel_Msg);
                else
                    vel_Msg.Linear.X = 0;
                    vel_Msg.Angular.Z = 0;
                    send(obj.velocity_publisher,vel_Msg);
                end    
            end   
            
            %state equation ...... e.g xDot = Ax + Bu (for linear systems). 
            xDot = [obj.velocity_magnitude*cos(x(3));
                    obj.velocity_magnitude*sin(x(3));
                    u(1);
                    obj.velocity_magnitude*cos(x(3)-x(5));
                    -obj.velocity_magnitude*sin(x(3)-x(5))/x(4)];                
        end
        
        function y = h(obj,t,x,varargin)
        
            % Subscriber read position of the vehicle the vehicle;
            turtle_position = receive(obj.position_subscriber,10);           
            
            % bounding theta of turtle between -pi to pi
            if( turtle_position.Theta > 3.14 )
                turtle_position.Theta = turtle_position.Theta - 2*3.14;
            end
            if( turtle_position.Theta < -3.14 )
                turtle_position.Theta = turtle_position.Theta + 2*3.14;
            end
            
            %state equation ...... e.g, Y = Cx + Du (for linear systems).  
            y = double([turtle_position.X;
                 turtle_position.Y;
                 turtle_position.Theta;
                 sqrt((obj.target(1) - turtle_position.X)^2 + (obj.target(2) - turtle_position.Y)^2);
                 (atan2((obj.target(2) - turtle_position.Y),(obj.target(1) - turtle_position.X)))]);
             
            disp('---taking output feedback---'); 
            obj.count = obj.count + 1;        
        end            
    end    
end