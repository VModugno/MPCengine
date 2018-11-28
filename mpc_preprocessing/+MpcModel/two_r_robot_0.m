%% two R robot
% name of the enviroment which the current model represents
env_name ="TwoRRobot";
%two R robot model
A_cont        = [0 0 1 0 ; 0 0 0 1 ; 0 0 0 0; 0 0 0 0];
B_cont        = [0 0; 0 0; 1 0; 0 1];
C_cont        = eye(4);%[1 0 0 0; 0 1 0 0];
% with this flag i tell the MPC constructor if the matrix has already been discretized or not      
discretized   = false;
%% init state
init_state    = [pi; pi/2; 0; 0];
%% cart pole state and control bounds  (for MPC)
maxOutput     = [10;10;10;10];
maxInput      = [20;20];
%% 2 r robot gains
state_gain    = 100;    % penalty error on the state
control_cost  = 1;  
%% predictive windows (it is useful for mutable constraints)
N             = 30;
%% here i define if the model is fixed or ltv
type          = "fixed"; 
%% here i define is the model is with mutable constraints or not
mutable_constr = [];
