%% model short description

% this model represent a simple 2r robot. in order to make it work we need
% to specify that we need a feedback linearization for the controller
% mainly use for the paylaod estimation problem with rls and integral
% sliding mode. it works both with regulation and tracking


%% two R robot
% name of the enviroment which the current model represents
env_name ="TwoRRobot";
% control step used inside the controller in general different from time step for integration 
internal_dt = 0.01; 
%two R robot model
A_cont        = [0 0 1 0 ; 0 0 0 1 ; 0 0 0 0; 0 0 0 0];
B_cont        = [0 0; 0 0; 1 0; 0 1];
C_cont        = eye(4);%[1 0 0 0; 0 1 0 0];
% with this flag i tell the MPC constructor if the matrix has already been discretized or not      
discretized   = false;
% with this i require to do the feedback linearization (by default is false)
feedback_lin  = true;
%% init state
init_state    = [pi; pi/2; 0; 0];
%% cart pole state and control bounds  (for MPC)
maxOutput     = [10;10;10;10];
maxInput      = [20;20];
%% 2 r robot gains
state_gain    = 100;    % penalty error on the state
control_cost  = 1;  
%% predictive windows 
N             = 20;
%% here i define if the model is fixed or ltv
type          = "fixed"; 
%% here i define is the model is with mutable constraints or not
mutable_constr = [];
%% function list
function_list.propagationModel = "std";
function_list.costFunc         = "std";      
function_list.constrW          = "walking";      
function_list.constrG          = "std";    
function_list.constrS          = "std"; 
%% here i define the trajectory (only for tracking only)
% sin traj
q1des_t  = pi/2*sin(t);
q2des_t  = pi/3*cos(t);
dq1des_t = pi/2*cos(t);
dq2des_t = -pi/3*sin(t);

x_des_model    = [q1des_t;q2des_t;dq1des_t;dq2des_t];

