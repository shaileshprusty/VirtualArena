classdef turtlesim_PP_RealVehicleROS < CtSystem

    properties
        target
        vm      % Attacker Velocity Magnitude
        attacker_position_subscriber
        attacker_velocity_publisher
    end
    
    methods
        
        function obj = turtlesim_PP_RealVehicleROS(target, ...
                                                   vm, ...
                                                   attacker_position_subscriber, ...
                                                   attacker_velocity_publisher ...
                                                   )
                                      
            obj = obj@CtSystem('nx',5,'nu',1,'ny',5);
            obj.target = target;
            obj.vm = vm;
            obj.attacker_position_subscriber = attacker_position_subscriber;
            obj.attacker_velocity_publisher = attacker_velocity_publisher;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            %% Publisher to send velocity to the vehicle;
            
            attacker_vel_Msg = rosmessage(obj.attacker_velocity_publisher);
            
            attacker_position = receive(obj.attacker_position_subscriber,10);
            
            distance = sqrt((attacker_position.X-obj.target(1))*(attacker_position.X-obj.target(1)) + (attacker_position.Y-obj.target(2))*(attacker_position.Y-obj.target(2)));
            
            if (distance >= 0.4)   
                attacker_vel_Msg.Linear.X = obj.vm;
                attacker_vel_Msg.Angular.Z = u(1);
%                 send(obj.velocity_publisher,vel_Msg);
            else
                attacker_vel_Msg.Linear.X = 0;
                attacker_vel_Msg.Angular.Z = 0;
%                 send(obj.velocity_publisher,vel_Msg);
            end    

%             attacker_vel_Msg.Linear.X = obj.vm;
%             attacker_vel_Msg.Angular.Z = u(1);
            
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

            attacker_position = receive(obj.attacker_position_subscriber,10);
            
            y = double([attacker_position.X;
                 attacker_position.Y;
            	 sqrt((obj.target(1) - attacker_position.X)^2 + (obj.target(2) - attacker_position.Y)^2);
                 atan2(obj.target(2) - attacker_position.Y, obj.target(1) - attacker_position.X);
                 attacker_position.Theta]);
        
        end
    
    end
    
end