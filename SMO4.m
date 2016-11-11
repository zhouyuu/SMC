function [sys,x0,str,ts] = SMO4(t,x,u,flag)
switch flag,
case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);
case 3,
    sys=mdlOutputs(t,x,u);
case {2,4,9}
    sys=[];
otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end
function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 4;
sizes.NumInputs      = 4;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 0;
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [];

function sys=mdlOutputs(t,x,u)

z0=u(1);
z1=u(2);
C=u(3); %input
tt=u(4);
e1=z0-C;
%e2=z1-v0;

p=5;q=9;
x=1;y=2;
g=50;
k=60;
v0=-g*abs(z0-C)^(x/y)*sign(z0-C)+z1;
z2=-k*abs(z1-v0)^(p/q)*sign(z1-v0);

sys(1)=v0;    
sys(2)=z2;
sys(3)=e1;
sys(4)=z1-11*cos(11*tt); 
