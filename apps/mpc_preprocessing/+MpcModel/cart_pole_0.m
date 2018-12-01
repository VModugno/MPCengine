%% cart pole model 
% name of the enviroment which the current model represents
env_name ="CartPole";
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