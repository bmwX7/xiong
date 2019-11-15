function [sys,x0,str,ts] = xiong1(t,x,u,flag)

%dispatch the flag,the switch fuction controls the calls to 
%s-function routines at each simulation stage.
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
      
    [sys,x0,str,ts,]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

 
  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  % case 9,
    %sys=mdlTerminate(t,x,u);
  
  case {2,4,9}
        sys=[];

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 3;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 3;
sizes.NumInputs      = 3;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [108 66.65 428];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state


% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

A=[0;-0.016*(x(1))^(9/8)-0.1*x(2);0.002235*x(1)];
B=[0.9 -0.0018*(x(1))^(9/8) -0.15;0 0.073*(x(1))^(9/8) 0;0 -0.01294*x(1) 1.6588];
sys = A+B*u;

% end mdlDerivatives



%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

a=((1-0.001538*x(3))*(0.8*x(1)-25.6))/(x(3)*(1.0394-0.0012304*x(1)));
q=(0.854*u(2)-0.147)*x(1)+45.59*u(1)-2.514*u(3)-2.096;
sys =[x(1);x(2);0.05*(0.13073*x(3)+100*a+(q/9-67.975))];

% end mdlOutputs


%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
