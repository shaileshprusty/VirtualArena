clc; close all; clear all;

dt = 0.2;
vm=4;
vt=2;
vd=4;
K=2;
N=3;
%% Unicycle Model
sys = ICtSystem(...
    'StateEquation', @(t,x,u,varargin) [
    vm*cos(x(9));
    vm*sin(x(9));
    vd*cos(u(1));
    vd*sin(u(1));
    vt*cos(u(2));
    vt*sin(u(2));
    vt*cos(u(2)-x(8))-vm*cos(x(9)-x(8));
    (vt*sin(u(2)-x(8))-vm*sin(x(9)-x(8)))/x(7);
    ((((-1).^floor(ceil(t/5)))+1)/2)*(N*((vt*sin(u(2)-x(8))-vm*sin(x(9)-x(8)))/x(7)))+(1-((((-1).^floor(ceil(t/5)))+1)/2))*(((-K*(x(9)-x(8)))/vm))],...
    'nx',9,'nu',2 ...
);
a1=0;
a2=0;
d1=150;
d2=200;
t1=100;
t2=50;
alm=0;
R=sqrt((t1-a1)^2+(t2-a2)^2);
Th=atan2((t2-a2),(t1-a1));
%sys.initialCondition = [0;0;100;150;50;50;70.72;0.78;0];
sys.initialCondition = [a1;a2;d1;d2;t1;t2;R;Th;alm];

% auxiliaryControlLaw =  TrackingController_ECC13(... % <<< attached to the realSystem
%     @(t) 10*[sin(0.1*t); cos(0.1*t)] , ... % c
%     @(t)    [cos(0.1*t);-sin(0.1*t)] , ... % cDot
%     eye(2)                           , ... % K
%     [1;0] );
% 
% e = @(t,x)auxiliaryControlLaw.computeError(t,x);

mpcOp = ICtMpcOp( ...
    'System'               , sys,...
    'HorizonLength'        , 10*dt,...
    'StageCost'            , @(t,x,u,varargin)-((x(5)-x(1))^2+(x(6)-x(2))^2)+(x(3)-x(1))^2+(x(4)-x(2))^2);
%     'TerminalCost'         , @(t,x,varargin) 0.3333*(e(t,x)'* e(t,x))^(3/2)...
%     );

dtMpcOp      = DiscretizedMpcOp(mpcOp,dt);

dtRealSystem = DiscretizedSystem(sys,dt);

dtRealSystem.controller = MpcController(...
    'MpcOp'       , dtMpcOp ,...
    'MpcOpSolver' , FminconMpcOpSolver('MpcOp', dtMpcOp,'UseSymbolicEvaluation',0) ...
    );

va = VirtualArena(dtRealSystem,...
    'StoppingCriteria'  , @(t,sysList)((dtRealSystem.x(3)-dtRealSystem.x(1))^2+(dtRealSystem.x(4)-dtRealSystem.x(2))^2)<1 || ((dtRealSystem.x(5)-dtRealSystem.x(1))^2+(dtRealSystem.x(6)-dtRealSystem.x(2))^2)<1,...
    'PlottingStep'      , 1/dt, ...
    'StepPlotFunction'  , @M2StepPlotFunction ...
    );

log = va.run();
