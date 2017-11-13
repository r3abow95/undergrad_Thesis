% NMPC Inverted Pendulum Controller design using ACADO Toolkit
%   Author: Alvin Reabow [RBWALV001]
%   Date: November 2017
% The ACADO toolkit was used to formulate the optimal control problem. The
% ACADO integrators were used to solve the control problem.
%====================================================================================

clc;
clear all;
close all;

Ts = 0.025;
EXPORT = 1;
COMPILE = 1;

if ~COMPILE % this is the simulink bus information necessary to perform the simulation
    load sim_data.mat ACADOdata ACADOinput ACADOoutput
end

DifferentialState x1 theta v1 dtheta;
Control F;
Disturbance R;

m1=0.009;
m0=0.058;
L1=0.225;
L0=0.09;
g=9.81;
J0=1.4316e-4;  
J1=2.078867e-4;

d0=1.5897e-3;         
d1=1.773e-6;         

Kt = 0.05722; 
Kg = 13/18;
Kf = 0.05722;
Ra = 17;

%% Differential Equation
    
f = [   dot(x1) - v1 == 0; ...
        dot(theta) - dtheta == 0; ...
        (v1*Kg^2*Kt^2)/Ra - (F*Kg*Kt)/Ra + dot(v1)*(m1*L0^2 + (m1*sin(theta)^2*L1^2)/4 + J0) + v1*((dtheta*m1*sin(2*theta)*L1^2)/4 + d0) + (L0*L1*m1*dot(dtheta)*cos(theta))/2 - (L0*L1*dtheta^2*m1*sin(theta))/2 == 0; ...
        d1*dtheta + J1*dot(dtheta) - (L1*g*m1*sin(theta))/2 - (L1^2*m1*v1^2*cos(theta)*sin(theta))/4 + (L0*L1*m1*cos(theta)*dot(v1))/2 == 0 ];

h = [diffStates; controls];
hN = [diffStates];

%% SIMexport
acadoSet('problemname', 'sim');

numSteps = 2;
sim = acado.SIMexport( Ts );

sim.setModel(f);

sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2' );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );

if EXPORT
    sim.exportCode( 'export_SIM' );
end
if COMPILE    
    cd export_SIM
    make_acado_integrator('../integrate_pendulum')
    cd ..
end

%% MPCexport
acadoSet('problemname', 'mpc');

N = 80;
ocp = acado.OCP( 0.0, N*Ts, N );

W = acado.BMatrix(eye(length(h)));
WN = acado.BMatrix(eye(length(hN)));

ocp.minimizeLSQ( W, h );
ocp.minimizeLSQEndTerm( WN, hN );

xmin = -2; xmax = 2;
Fmin = -12; Fmax = 12;

ocp.subjectTo( xmin <= x1 <= xmax );
ocp.subjectTo( Fmin <= F <= Fmax );
ocp.subjectTo(R == 0.0 );
ocp.setModel(f);

mpc = acado.OCPexport( ocp );

mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'               );
mpc.set( 'LEVENBERG_MARQUARDT',         1e-4                );

if EXPORT
    mpc.exportCode( 'export_MPC' );
end
if COMPILE
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'export_MPC/qpoases3')
    
    cd export_MPC
    make_acado_solver_sfunction
    copyfile('acado_solver_sfun.mex*', '../')
    cd ..
end

%% PARAMETERS SIMULATION
X0 = [0 pi 0 0];
Xref = [0 0 0 0];
input.x = repmat(X0,N+1,1).';

Uref = zeros(N,1);
input.u = Uref.';

input.y = [repmat(X0,N,1) Uref].';
input.yN = X0.';

input.W = diag([5e-1 1 2e-3 2e-3 1e-2]);
input.WN = diag([5e-1 1 2e-3 2e-3]);

input.x0 = X0.';

%% SIMULATION LOOP

init.x = input.x(:).';
init.u = input.u(:).';
init.y = input.y(:).';
init.yN = input.yN(:).';
init.W = input.W(:).';
init.WN = input.WN(:).';
init.x0 = input.x0(:).';

save sim_data.mat ACADOdata ACADOinput ACADOoutput

