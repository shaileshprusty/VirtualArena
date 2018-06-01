classdef ardurover_MPC_RealVehicleROS < CtSystem

    properties
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
            attacker_angle = receive(obj.attacker_angle_subscriber,10);
            target_LatLon = receive(obj.target_LatLon_subscriber,10);
            target_angle = receive(obj.target_angle_subscriber,10);
            defender_LatLon = receive(obj.defender_LatLon_subscriber,10);
            defender_angle = receive(obj.defender_angle_subscriber,10);
            
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

            attacker_vel_Msg.Linear.X = obj.vm;
            attacker_vel_Msg.Angular.Z = x(9);
            target_vel_Msg.Linear.X = obj.vt;
            target_vel_Msg.Angular.Z = 0;
            defender_vel_Msg.Linear.X = obj.vd;
            defender_vel_Msg.Angular.Z = u(1);
            
            xDot = [obj.vm*cos(x(10));
                    obj.vm*sin(x(10));
                    obj.vd*cos(x(5));
                    obj.vd*sin(x(5));
                    u(1);
                    obj.vt*cos(target_angle.Data);
                    obj.vt*sin(target_angle.Data);
                    obj.vt*cos(target_angle.Data-x(9))-obj.vm*cos(x(10)-x(8));
                    (obj.vt*sin(target_angle.Data-x(9))-obj.vm*sin(x(10)-x(9)))/x(8);
                    ((((-1).^floor(ceil(t/obj.switchGuidanceLaw)))+1)/2)*(obj.N*((obj.vt*sin(target_angle.Data-x(9))-obj.vm*sin(x(10)-x(9)))/x(8)))+(1-((((-1).^floor(ceil(t/obj.switchGuidanceLaw)))+1)/2))*(((-obj.K*(x(10)-x(9)))/obj.vm))
                    ];

        
        end
        
        function y = h(obj,t,x,varargin)
        
            %% Subscriber to read the position of the vehicle

            attacker_LatLon = receive(obj.attacker_LatLon_subscriber,10);
            attacker_angle = receive(obj.attacker_angle_subscriber,10);
            target_LatLon = receive(obj.target_LatLon_subscriber,10);
            target_angle = receive(obj.target_angle_subscriber,10);
            defender_LatLon = receive(obj.defender_LatLon_subscriber,10);
            defender_angle = receive(obj.defender_angle_subscriber,10);
            
            y = [attacker_LatLon.Latitude;
                 attacker_LatLon.Longitude;
            	 defender_LatLon.Latitude;
                 defender_LatLon.Longitude;
                 defender_angle.Data;
                 target_LatLon.Latitude;
                 target_LatLon.Longitude;
            	 
                 
                 attacker_angle.Data;
                 ];
        
        end
    
    end
    
end