clear variables
close all
clc

% cart pole parameters (to the env class)
mCart = 0.1;
mPend = 0.1;
L = 0.5;
g = 9.8;
xCartMax = 5;
% Double integrator
A_cont = [0 1 0 0; 0 0 -mPend*g/mCart 0; 0 0 0 1; 0 0 (mCart+mPend)*g/(L*mCart) 0];
B_cont = [0; 1/mCart; 0; -1/(L*mCart)];
C_cont = [0 1 1 1];
maxOutput    = [10];
maxInput     = [20];
delta        = 0.1;     % (to the env class)
N            = 3;       % prediction window
state_gain   = 100;     % penalty error on the state
control_cost = 1;
type         = "fixed"; 
solver       = "QPoases";
% regulator 
controller = MpcGen.mpcRegulator(A_cont,B_cont,C_cont,maxInput,maxOutput,delta,N,state_gain,control_cost,type,solver);

controller.GenFunction();
