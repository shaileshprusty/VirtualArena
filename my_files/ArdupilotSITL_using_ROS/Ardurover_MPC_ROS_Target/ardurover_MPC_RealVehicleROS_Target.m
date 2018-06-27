classdef ardurover_MPC_RealVehicleROS_Target < CtSystem

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
        attacker_angular_velocity
        target_angle_Msg
        
        attacker_vel_Msg
        target_vel_Msg
        defender_vel_Msg
    end
    
    methods
        
        function obj = ardurover_MPC_RealVehicleROS_Target(N, ...
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
%             obj.target_angle_Msg = receive(obj.target_angle_subscriber,10);
%             obj.target_angle = obj.target_angle_Msg.Data;
            
            obj.attacker_vel_Msg = rosmessage(obj.attacker_velocity_publisher);
            obj.target_vel_Msg = rosmessage(obj.target_velocity_publisher);
            obj.defender_vel_Msg = rosmessage(obj.defender_velocity_publisher);

        end

        %%
        function xDot = f(obj,t,x,u,varargin)
          
            xDot = [obj.vm*cos(x(11));  %Velocity of Attacker in x-dircetion
                    obj.vm*sin(x(11));  %Velocity of Attacker in y-dircetion
                    obj.vd*cos(x(5));   %Velocity of Defender in x-dircetion
                    obj.vd*sin(x(5));   %Velocity of Defender in x-dircetion
                    u(1);               %Angular Velocity of Defender
                    obj.vt*cos(x(8));  %Velocity of Target in x-dircetion
                    obj.vt*sin(x(8));  %Velocity of Target in y-dircetion
                    u(2);
                    obj.vt*cos(x(8)-x(10))-obj.vm*cos(x(11)-x(10));  %Velocity along Attacker-Target LOS
                    (obj.vt*sin(x(8)-x(10))-obj.vm*sin(x(11)-x(10)))/x(9);   %Angular velocity of above equation
%                     ((((-1).^floor(ceil(t/obj.switchGuidanceLaw)))+1)/2)*(obj.N*((obj.vt*sin(target_angle.Data-x(9))-obj.vm*sin(x(10)-x(9)))/x(8)))+(1-((((-1).^floor(ceil(t/obj.switchGuidanceLaw)))+1)/2))*(((-obj.K*(x(10)-x(9)))/obj.vm))];  %Angular Velocity of Target
                    obj.N*((obj.vt*sin(x(8)-x(10))-obj.vm*sin(x(11)-x(10)))/x(9))];
                    
            obj.attacker_angular_velocity = obj.N*((obj.vt*sin(x(8)-x(10))-obj.vm*sin(x(11)-x(10)))/x(9));
            
            
             %asdf1= toc(asdf);
             %disp('--time aken by f--');
             %disp(asdf1);
        end
        
        function y = h(obj,t,x,varargin)
        
            qwerty = tic;
            %% Subscriber to read the position of the vehicle

            %attacker_LatLon = receive(obj.attacker_LatLon_subscriber,10);
            %attacker_angle = receive(obj.attacker_angle_subscriber,10);
            attacker_Lat = obj.attacker_LatLon_subscriber.LatestMessage.Latitude;
            attacker_Lon = obj.attacker_LatLon_subscriber.LatestMessage.Longitude;
            attacker_angle = obj.attacker_angle_subscriber.LatestMessage.Data;
            
            qwerty1 = toc(qwerty);
            disp(['-attacker receive time-',num2str(qwerty1)]);

            qwerty2 = tic;
%             target_LatLon = receive(obj.target_LatLon_subscriber,10);
%             target_angle = receive(obj.target_angle_subscriber,10);
            target_Lat = obj.target_LatLon_subscriber.LatestMessage.Latitude;
            target_Lon = obj.target_LatLon_subscriber.LatestMessage.Longitude;
            target_angle = obj.target_angle_subscriber.LatestMessage.Data;
            

            qwerty3 = toc(qwerty2);
%             disp(['-target receive time-',num2str(qwerty3)]);
            qwerty4 = tic;

%             defender_LatLon = receive(obj.defender_LatLon_subscriber,10);
%             defender_angle = receive(obj.defender_angle_subscriber,10);
           
            defender_Lat = obj.defender_LatLon_subscriber.LatestMessage.Latitude;
            defender_Lon = obj.defender_LatLon_subscriber.LatestMessage.Longitude;
            defender_angle = obj.defender_angle_subscriber.LatestMessage.Data;
          
          
            qwerty5 = toc(qwerty4);
%             disp(['-defender receive time-',num2str(qwerty5)]);
            

            %% bounding theta of rover between -pi to pi

            attacker_angle = 90 - attacker_angle;
            defender_angle = 90 - defender_angle;
            target_angle = 90 - target_angle;

            
            %% Convert to UTM
            
            [attacker_utmX, attacker_utmY] = deg2utm(attacker_Lat, attacker_Lon);
            [target_utmX, target_utmY] = deg2utm(target_Lat, target_Lon);
            [defender_utmX, defender_utmY] = deg2utm(defender_Lat, defender_Lon);

            %% ......                               

            
            y = double([attacker_utmX;
                        attacker_utmY;
                        defender_utmX;
                        defender_utmY;
                        deg2rad(defender_angle);
                        target_utmX;
                        target_utmY;
                        deg2rad(target_angle)
                        sqrt((target_utmX - attacker_utmX)^2 + (target_utmY - attacker_utmY)^2);
                        atan2(target_utmY - attacker_utmY, target_utmX - attacker_utmX);
                        deg2rad(attacker_angle)]);
        
             obj.count = obj.count + 1;
        end
        
        function pub(obj,t,x,u,varargin)
            time3 = tic;
            attacker_utmX = x(1);
            attacker_utmY = x(2);
            target_utmX = x(6);
            target_utmY = x(7);
            defender_utmX = x(3);
            defender_utmY = x(4);

            attacker_target_distance = sqrt((attacker_utmX - target_utmX)^2 + (attacker_utmY - target_utmY)^2);
            defender_attacker_distance = sqrt((defender_utmX - attacker_utmX)^2 + (defender_utmY - attacker_utmY)^2);
            
            if (obj.count > 3)
                if (attacker_target_distance >= 0.7 && defender_attacker_distance >= 0.7)   
                    obj.attacker_vel_Msg.Linear.X = obj.vm;
                    disp('attacker angular velocity');
                    disp(obj.attacker_angular_velocity);
                    obj.attacker_vel_Msg.Angular.Z = obj.attacker_angular_velocity;
                    obj.target_vel_Msg.Linear.X = obj.vt;
                    obj.target_vel_Msg.Angular.Z = 0;
                    obj.defender_vel_Msg.Linear.X = obj.vd;
                    obj.defender_vel_Msg.Angular.Z = u(1)^(0.5);
                    send(obj.attacker_velocity_publisher, obj.attacker_vel_Msg);
                    send(obj.target_velocity_publisher, obj.target_vel_Msg);
                    send(obj.defender_velocity_publisher, obj.defender_vel_Msg);
                else
                    obj.attacker_vel_Msg.Linear.X = 0;
                    obj.attacker_vel_Msg.Angular.Z = 0;
                    obj.target_vel_Msg.Linear.X = 0;
                    obj.target_vel_Msg.Angular.Z = 0;
                    obj.defender_vel_Msg.Linear.X = 0;
                    obj.defender_vel_Msg.Angular.Z = 0;
                    while (1)
%                         if (attacker_target_distance <= 0.5)
%                            disp('attacker reached target');
%                         end
%                         if (defender_attacker_distance <= 0.5)
%                            disp('defender reached attacker');
%                         end
                        disp('attacker_target_distance:');
                        disp(attacker_target_distance);
                        disp('defender_attacker_distance:');
                        disp(defender_attacker_distance);
                        disp();
                        send(obj.attacker_velocity_publisher, obj.attacker_vel_Msg);
                        send(obj.target_velocity_publisher, obj.target_vel_Msg);
                        send(obj.defender_velocity_publisher, obj.defender_vel_Msg);
                    end
                end
            end
            
            time4 = toc(time3);
%             disp('--- pub time ---')
%             disp(time4);
        
        end
    
    end
    
end