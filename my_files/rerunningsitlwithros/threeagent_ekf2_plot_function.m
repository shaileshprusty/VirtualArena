function h = threeagent_ekf2_plot_function(sysList,log,plot_handles,k)

logX = log{1}.stateTrajectory(:,1:k); hold on;
h = plot(logX(1,:),logX(2,:),logX(3,:),logX(4,:),logX(5,:),logX(6,:));

logHatX = log{1}.observerStateTrajectory(:,1:k); hold on;
h2    = plot(logHatX(1,:),logHatX(2,:),logHatX(3,:),logHatX(4,:),logHatX(5,:),logHatX(6,:),'--');


% logX    = log{1}.stateTrajectory(:,1:k); hold on;
% logHatX = log{1}.observerStateTrajectory(:,1:k); hold on;
% h(1)    = plot(logX(1,:)   ,logX(2,:));
% h(2)    = plot(logHatX(1,:),logHatX(2,:),'--');
% 
% logX2    = log{2}.stateTrajectory(:,1:k); hold on;
% logHatX2 = log{2}.observerStateTrajectory(:,1:k); hold on;
% h2(1)    = plot(logX2(1,:)   ,logX2(2,:));
%h2(2)    = plot(logHatX2(1,:),logHatX2(2,:),'--');

grid on

end
