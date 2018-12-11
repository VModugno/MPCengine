%% model short description

% this model represent a simple cart pole problem linearized around the up
% up position. it is mainly used for testing the mpc controller for
% regulation problem. it can be used without feedback lin if the initial
% position is around a up up position. we do not ahve implemented andy
% fucntion for feedback linearization so far


%% cart pole model 
% name of the enviroment which the current model represents
env_name ="CartPole";
% control step used inside the controller in general different from time step for integration 
internal_dt = 0.05; 
%cart pole parameters (to the env class) (TODO add read parameters from file)
mCart    = 0.1;
mPend    = 0.1;
L        = 0.5;
g        = 9.8;
xCartMax = 5;
% Double integrator
A_cont = [0 1 0 0; 0 0 -mPend*g/mCart 0; 0 0 0 1; 0 0 (mCart+mPend)*g/(L*mCart) 0];
B_cont = [0; 1/mCart; 0; -1/(L*mCart)];
C_cont = [1 0 0 0;
          0 0 1 0;
          0 0 0 1];
% with this flag i tell the MPC constructor if the matrix has already been discretized or not      
discretized = false;
% with this i require to do the feedback linearization (by default is false)
feedback_lin  = false;
%% init state
init_state = [0; 0; pi/8; 0];
%% cart pole state and control bounds  
maxOutput    = [10;100;100];
maxInput     = [100];
%% gains
state_gain   = 100;    % penalty error on the state
control_cost = 1; 
%% predictive windows (it is useful for mutable constraints)
N            = 30;  
%% here i define if the model is fixed or ltv
type         = "fixed"; % if the model is type ltv i need to specify the system matrix already using symbolic varialbes
%% here i define if the model is with variable constraints or not
mutable_constr = [];