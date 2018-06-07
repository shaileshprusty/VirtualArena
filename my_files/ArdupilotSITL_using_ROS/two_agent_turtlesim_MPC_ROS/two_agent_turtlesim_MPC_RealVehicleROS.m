classdef two_agent_turtlesim_MPC_RealVehicleROS < CtSystem

    properties
         
        count = 0
        vm      % Attacker Velocity Magnitude
        vt      % Target Velocity Magnitude
        target_position_subscriber
        target_velocity_publisher
        attacker_position_subscriber
        attacker_velocity_publisher

    end
    
    methods
        
        function obj = two_agent_turtlesim_MPC_RealVehicleROS(target_position_subscriber, ...
                                                              target_velocity_publisher, ...
                                                              attacker_position_subscriber, ...
                                                              attacker_velocity_publisher, ...
                                                              vm, ...
                                                              vt ...
                                                              )
            
            obj = obj@CtSystem('nx',7,'nu',1,'ny',7);            
            obj.target_position_subscriber = target_position_subscriber;
            obj.target_velocity_publisher = target_velocity_publisher;
            obj.attacker_position_subscriber = attacker_position_subscriber;            
            obj.attacker_velocity_publisher = attacker_velocity_publisher;
            obj.vm = vm;
            obj.vt = vt;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            % Publisher send u to the vehicle;
            target_velocity_Msg = rosmessage(obj.target_velocity_publisher);
            attacker_velocity_Msg = rosmessage(obj.attacker_velocity_publisher);
            
            target_position = receive(obj.target_position_subscriber,10);
            attacker_position = receive(obj.attacker_position_subscriber,10);

            distance = sqrt((target_position.X-attacker_position.X)^2 + (target_position.Y-attacker_position.Y)^2);    %distance between target and attacker.
            
            if obj.count > 2             %to avoid initial random values.
                disp('--------publishing--------');
                if (distance >= 0.2)                    
                    target_velocity_Msg.Linear.X = obj.vt;
                    attacker_velocity_Msg.Linear.X = obj.vm;
                    target_velocity_Msg.Angular.Z = 0;
                    attacker_velocity_Msg.Angular.Z = u(1);
                    disp(u(1));
                else
                    target_velocity_Msg.Linear.X = 0;
                    attacker_velocity_Msg.Linear.X = 0;
                    target_velocity_Msg.Angular.Z = 0;
                    attacker_velocity_Msg.Angular.Z = 0;
                    disp(0);
                end    
                send(obj.attacker_velocity_publisher,attacker_velocity_Msg);
                send(obj.target_velocity_publisher,target_velocity_Msg);
            end   
            
            %state equation ...... e.g xDot = Ax + Bu (for linear systems). 
            xDot = [obj.vm*cos(x(3));
                    obj.vm*sin(x(3));
                    u(1);
                    obj.vt*cos(0);
                    obj.vt*sin(0);
                    -obj.vt*cos(-x(7))+obj.vm*cos(x(3)-x(7));
                    (obj.vt*sin(-x(7))-obj.vm*sin(x(3)-x(7)))/x(6)];                
        end
        
        function y = h(obj,t,x,varargin)
        
            % Subscriber read position of the vehicle the vehicle;
            target_position = receive(obj.target_position_subscriber,10);
            attacker_position = receive(obj.attacker_position_subscriber,10);           
            
            % bounding theta of turtle between -pi to pi
            if( attacker_position.Theta > 3.14 )
                attacker_position.Theta = attacker_position.Theta - 2*3.14;
            end
            if( attacker_position.Theta < -3.14 )
                attacker_position.Theta = attacker_position.Theta + 2*3.14;
            end
            
            %state equation ...... e.g, Y = Cx + Du (for linear systems).  
            y = double([attacker_position.X;
                 attacker_position.Y;
                 attacker_position.Theta;
                 target_position.X;
                 target_position.Y;
                 sqrt((target_position.X - attacker_position.X)^2 + (target_position.Y - attacker_position.Y)^2);
                 (atan2((target_position.Y - attacker_position.Y),(target_position.X - attacker_position.X)))]);
             
            disp('---taking output feedback---'); 
            obj.count = obj.count + 1;        
        end            
    end    
end