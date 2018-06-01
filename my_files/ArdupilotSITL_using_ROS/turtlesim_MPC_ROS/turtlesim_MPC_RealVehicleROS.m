classdef turtlesim_MPC_RealVehicleROS < CtSystem

    properties
        N
        K
        switchGuidanceLaw
        vm      % Attacker Velocity Magnitude
        vt      % Target Velocity Magnitude
        vd      % Defender Velocity Magnitude
        attacker_position_subscriber
        attacker_velocity_publisher
        target_position_subscriber
        target_velocity_publisher
        defender_position_subscriber
        defender_velocity_publisher
    end
    
    methods
        
        function obj = turtlesim_MPC_RealVehicleROS(N, ...
                                          K, ...
                                          switchGuidanceLaw, ...
                                          vm, ...
                                          vt, ...
                                          vd, ...
                                          attacker_position_subscriber, ...
                                          attacker_velocity_publisher, ...
                                          target_position_subscriber, ...
                                          target_velocity_publisher, ...
                                          defender_position_subscriber, ...
                                          defender_velocity_publisher ...
                                          )
                                      
            obj = obj@CtSystem('nx',10,'nu',1,'ny',10);
            obj.N = N;
            obj.K = K;
            obj.switchGuidanceLaw = switchGuidanceLaw;
            obj.vm = vm;
            obj.vt = vt;
            obj.vd = vd;
            obj.attacker_position_subscriber = attacker_position_subscriber;
            obj.attacker_velocity_publisher = attacker_velocity_publisher;
            obj.target_position_subscriber = target_position_subscriber;
            obj.target_velocity_publisher = target_velocity_publisher;
            obj.defender_position_subscriber = defender_position_subscriber;
            obj.defender_velocity_publisher = defender_velocity_publisher;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            %% Publisher to send velocity to the vehicle;
            
            attacker_vel_Msg = rosmessage(obj.attacker_velocity_publisher);
            target_vel_Msg = rosmessage(obj.target_velocity_publisher);
            defender_vel_Msg = rosmessage(obj.defender_velocity_publisher);
            
            attacker_position = receive(obj.attacker_position_subscriber,10);
            target_position = receive(obj.target_position_subscriber,10);
            defender_position = receive(obj.defender_position_subscriber,10);
            
%             position_params = [rover_LatLon.Latitude;rover_LatLon.Longitude;rover_angle.Data];  %Convert msg variable to matrix
%             distance = sqrt((position_params(1)-obj.target(1))*(position_params(1)-obj.target(1)) + (position_params(2)-obj.target(2))*(position_params(2)-obj.target(2)));
%             
%             if (distance >= 0.0002)   
%                 vel_Msg.Linear.X = obj.velocity_magnitude;
%                 vel_Msg.Angular.Z = u(1);
%                 send(obj.velocity_publisher,vel_Msg);
%             else
%                 vel_Msg.Linear.X = 0;
%                 vel_Msg.Angular.Z = 0;
%                 send(obj.velocity_publisher,vel_Msg);
%             end    
            
            
            
            xDot = [obj.vm*cos(x(10));  %Velocity of Attacker in x-dircetion
                    obj.vm*sin(x(10));  %Velocity of Attacker in y-dircetion
                    obj.vd*cos(x(5));   %Velocity of Defender in x-dircetion
                    obj.vd*sin(x(5));   %Velocity of Defender in x-dircetion
                    u(1);               %Angular Velocity of Defender
                    obj.vt*cos(target_position.Theta);  %Velocity of Target in x-dircetion
                    obj.vt*sin(target_position.Theta);  %Velocity of Target in y-dircetion
                    obj.vt*cos(target_position.Theta-x(9))-obj.vm*cos(x(10)-x(8));  %Velocity along Attacker-Target LOS
                    (obj.vt*sin(target_position.Theta-x(9))-obj.vm*sin(x(10)-x(9)))/x(8);   %Angular velocity of above equation
                    ((((-1).^floor(ceil(t/obj.switchGuidanceLaw)))+1)/2)*(obj.N*((obj.vt*sin(target_position.Theta-x(9))-obj.vm*sin(x(10)-x(9)))/x(8)))+(1-((((-1).^floor(ceil(t/obj.switchGuidanceLaw)))+1)/2))*(((-obj.K*(x(10)-x(9)))/obj.vm))];  %Angular Velocity of Target

            attacker_vel_Msg.Linear.X = obj.vm;
            attacker_vel_Msg.Angular.Z = xDot(10);
            target_vel_Msg.Linear.X = obj.vt;
            target_vel_Msg.Angular.Z = 0;
            defender_vel_Msg.Linear.X = obj.vd;
            defender_vel_Msg.Angular.Z = u(1);
            
            send(obj.attacker_velocity_publisher, attacker_vel_Msg);
            send(obj.target_velocity_publisher, target_vel_Msg);
            send(obj.defender_velocity_publisher, defender_vel_Msg);
        
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber to read the position of the vehicle

            attacker_position = receive(obj.attacker_position_subscriber,10);
            target_position = receive(obj.target_position_subscriber,10);
            defender_position = receive(obj.defender_position_subscriber,10);
            
            y = double([attacker_position.X;
                 attacker_position.Y;
            	 defender_position.X;
                 defender_position.Y;
                 defender_position.Theta;
                 target_position.X;
                 target_position.Y;
            	 sqrt((target_position.X - attacker_position.X)^2 + (target_position.Y - attacker_position.Y)^2);
                 atan2(target_position.Y - attacker_position.Y, target_position.X - attacker_position.X);
                 attacker_position.Theta]);
        
        end
    
    end
    
end