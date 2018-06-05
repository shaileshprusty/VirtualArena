classdef ardurover_MPC_RealVehicleROS < CtSystem

    properties
        count = 0
        N
        K
        switchGuidanceLaw
        vm      % Attacker Velocity Magnitude
        vt      % Target Velocity Magnitude
        vd      % Defender Velocity Magnitude
        attacker_LatLon_subscriber
        attacker_angle_subscriber
        attacker_velocity_publisher
        target_LatLon_subscriber
        target_angle_subscriber
        target_velocity_publisher
        defender_LatLon_subscriber
        defender_angle_subscriber
        defender_velocity_publisher
    end
    
    methods
        
        function obj = ardurover_MPC_RealVehicleROS(N, ...
                                          K, ...
                                          switchGuidanceLaw, ...
                                          vm, ...
                                          vt, ...
                                          vd, ...
                                          attacker_LatLon_subscriber, ...
                                          attacker_angle_subscriber, ...
                                          attacker_velocity_publisher, ...
                                          target_LatLon_subscriber, ...
                                          target_angle_subscriber, ...
                                          target_velocity_publisher, ...
                                          defender_LatLon_subscriber, ...
                                          defender_angle_subscriber, ...
                                          defender_velocity_publisher ...
                                          )
                                      
            obj = obj@CtSystem('nx',10,'nu',1,'ny',10);
            obj.N = N;
            obj.K = K;
            obj.switchGuidanceLaw = switchGuidanceLaw;
            obj.vm = vm;
            obj.vt = vt;
            obj.vd = vd;
            obj.attacker_LatLon_subscriber = attacker_LatLon_subscriber;
            obj.attacker_angle_subscriber = attacker_angle_subscriber;
            obj.attacker_velocity_publisher = attacker_velocity_publisher;
            obj.target_LatLon_subscriber = target_LatLon_subscriber;
            obj.target_angle_subscriber = target_angle_subscriber;
            obj.target_velocity_publisher = target_velocity_publisher;
            obj.defender_LatLon_subscriber = defender_LatLon_subscriber;
            obj.defender_angle_subscriber = defender_angle_subscriber;
            obj.defender_velocity_publisher = defender_velocity_publisher;
        end
        
        function xDot = f(obj,t,x,u,varargin)
            
            %% Publisher to send velocity to the vehicle;
            
            attacker_vel_Msg = rosmessage(obj.attacker_velocity_publisher);
            target_vel_Msg = rosmessage(obj.target_velocity_publisher);
            defender_vel_Msg = rosmessage(obj.defender_velocity_publisher);
            
            attacker_LatLon = receive(obj.attacker_LatLon_subscriber,10);
%             attacker_angle = receive(obj.attacker_angle_subscriber,10);
            target_LatLon = receive(obj.target_LatLon_subscriber,10);
            target_angle = receive(obj.target_angle_subscriber,10);
            defender_LatLon = receive(obj.defender_LatLon_subscriber,10);
%             defender_angle = receive(obj.defender_angle_subscriber,10);

            attacker_target_distance = sqrt((attacker_LatLon.Latitude - target_LatLon.Longitude)^2 + (attacker_LatLon.Longitude - target_LatLon.Longitude)^2);
            defender_attacker_distance = sqrt((defender_LatLon.Latitude - attacker_LatLon.Latitude)^2 + (defender_LatLon.Latitude - attacker_LatLon.Longitude)^2);
            
            xDot = [obj.vm*cos(x(10));  %Velocity of Attacker in x-dircetion
                    obj.vm*sin(x(10));  %Velocity of Attacker in y-dircetion
                    obj.vd*cos(x(5));   %Velocity of Defender in x-dircetion
                    obj.vd*sin(x(5));   %Velocity of Defender in x-dircetion
                    u(1);               %Angular Velocity of Defender
                    obj.vt*cos(target_angle.Data);  %Velocity of Target in x-dircetion
                    obj.vt*sin(target_angle.Data);  %Velocity of Target in y-dircetion
                    obj.vt*cos(target_angle.Data-x(9))-obj.vm*cos(x(10)-x(9));  %Velocity along Attacker-Target LOS
                    (obj.vt*sin(target_angle.Data-x(9))-obj.vm*sin(x(10)-x(9)))/x(8);   %Angular velocity of above equation
%                     ((((-1).^floor(ceil(t/obj.switchGuidanceLaw)))+1)/2)*(obj.N*((obj.vt*sin(target_angle.Data-x(9))-obj.vm*sin(x(10)-x(9)))/x(8)))+(1-((((-1).^floor(ceil(t/obj.switchGuidanceLaw)))+1)/2))*(((-obj.K*(x(10)-x(9)))/obj.vm))];  %Angular Velocity of Target
                    obj.N*((obj.vt*sin(target_angle.Data-x(9))-obj.vm*sin(x(10)-x(9)))/x(8))];
                    
            if (obj.count > 2)
                if (attacker_target_distance >= 0.2 && defender_attacker_distance >= 0.2)   
                    attacker_vel_Msg.Linear.X = obj.vm;
                    attacker_vel_Msg.Angular.Z = xDot(10);
                    target_vel_Msg.Linear.X = obj.vt;
                    target_vel_Msg.Angular.Z = 0;
                    defender_vel_Msg.Linear.X = obj.vd;
                    defender_vel_Msg.Angular.Z = u(1);
                else
                    attacker_vel_Msg.Linear.X = 0;
                    attacker_vel_Msg.Angular.Z = 0;
                    target_vel_Msg.Linear.X = 0;
                    target_vel_Msg.Angular.Z = 0;
                    defender_vel_Msg.Linear.X = 0;
                    defender_vel_Msg.Angular.Z = 0;
                end
            end
            
            send(obj.attacker_velocity_publisher, attacker_vel_Msg);
            send(obj.target_velocity_publisher, target_vel_Msg);
            send(obj.defender_velocity_publisher, defender_vel_Msg);
        
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber to read the position of the vehicle

            attacker_LatLon = receive(obj.attacker_LatLon_subscriber,10);
            attacker_angle = receive(obj.attacker_angle_subscriber,10);
            target_LatLon = receive(obj.target_LatLon_subscriber,10);
%             target_angle = receive(obj.target_angle_subscriber,10);
            defender_LatLon = receive(obj.defender_LatLon_subscriber,10);
            defender_angle = receive(obj.defender_angle_subscriber,10);

            if(attacker_angle.Data > 180)
                attacker_angle.Data = attacker_angle.Data - 2*180;
            end
            
            if(defender_angle.Data > 180)
                defender_angle.Data = defender_angle.Data - 2*180;
            end
            
            y = double([attacker_LatLon.Latitude;
                        attacker_LatLon.Longitude;
                        defender_LatLon.Latitude;
                        defender_LatLon.Longitude;
                        3.14*defender_angle.Data/180;
                        target_LatLon.Latitude;
                        target_LatLon.Longitude;
                        sqrt((target_LatLon.Latitude - attacker_LatLon.Latitude)^2 + (target_LatLon.Longitude - attacker_LatLon.Longitude)^2);
                        atan2(target_LatLon.Longitude - attacker_LatLon.Longitude, target_LatLon.Latitude - attacker_LatLon.Latitude);
                        3.14*attacker_angle.Data/180]);
        
             obj.count = obj.count + 1;
        end
    
    end
    
end