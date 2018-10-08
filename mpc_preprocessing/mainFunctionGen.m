clear variables
close all
clc


A_cont       = [0 0 1 0 ; 0 0 0 1 ; 0 0 0 0; 0 0 0 0];
B_cont       = [0 0; 0 0; 1 0; 0 1];
C_cont       = eye(4);%[1 0 0 0; 0 1 0 0];
maxOutput    = [10;10;10;10];
maxInput     = [20;20];
delta        = 0.1;
N            = 3;       % prediction window
state_gain   = 100;     % penalty error on the state
control_cost = 1;
type         = "fixed"; 
solver       = "QPoases";
% regulator 
controller = MpcGen.mpcRegulator(A_cont,B_cont,C_cont,maxInput,maxOutput,delta,N,state_gain,control_cost,type,solver);

controller.GenFunction();
