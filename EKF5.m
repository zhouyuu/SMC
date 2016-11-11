function [sys,x0,str,ts,simStateCompliance] = EKF5(t,x,u,flag)
global G H C Q R P K P0 z U m T;
P=[1 0 0 0;
   0 1 0 0;
   0 0 1e7 0;
   0 0 0 6e8];
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes


sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 4;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  =  [0;0;0;0];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [1e-6 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)
global G H C Q R P K P0 z U m T;
T=1e-6;
U=[u(3);u(4)];
m=[u(1);u(2)];
Q=[0.1,0,0,0;
   0,0.1,0,0;
   0,0,1e6,0;
   0,0,0,1e8];

R=[1,0;
   0,1];




C=[1,0,0,0;
   0,1,0,0];



H=[59.7*T,0;
   0,59.7*T;
   0,0;
   0,0]; 

G=[-328.3*T+1,0,5.67*T*sin(x(4)),0;
   0,-328.3*T+1,-5.67*T*cos(x(4)),0;
   0,0,1,0;
   0,0,T,1];



z=G*x+H*U;
P0=G*P*G'+Q;
K=P0*C'/(C*P0*C'+R);
P=P0-K*C*P0;
x=z+K*(m-C*z);
sys = x;


function sys=mdlOutputs(t,x,u)
global G H C Q R P K P0 z U m T;
sys=x;
% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1e-6;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
