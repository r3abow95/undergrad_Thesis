%% LQR Controller Build function
% Run this before running any Simulink or Matlab model of the RIP system
% Required to initialise controller gains and observer gains
% Author: Alvin Reabow
% Date: November 2017

initialise;
[A,B,C,D] = linmod('RIP1');			%linearise system model
C = [0 0 1 0];
D = [0];
sys = ss(A,B,C,D);

sys_d = c2d(sys,0.025,'zoh');

Q = [5 0 0 0 0;0 0.1 0 0 0;0 0 10 0 0;0 0 0 2 0; 0 0 0 0 1]; %state weighting function
R = 1; %control action weighting


K = lqi(sys_d,Q,R)							%State Gains
Ki = K(5);									%Integrator Gain

a = [sys_d.a zeros(4,1);-C 0];
b = [sys_d.b;0];
cl_poles = eig(a-b*K);
%select poles to be at least 4-10x faster than slowest C/L pole
P = [0.15 0.12 0.1 0.16];      %Observer poles
L = place(sys_d.a',[1 0 0 0;0 0 1 0]',P)'            %observer gains


%Swing-up controller parameters
k = 350;
n = 2;
