%% model short description
% 2d lip with both x and y direction plus the simple model for the feet
% movement both in x and y.
% in this model we employ a specific discretization for the lip model 
% and it is built in a way that we need to employ a regulator MPC


%% 2dx_or_y_lip (this system is already discretized)
%% Model 
% name of the enviroment which the current model represents
env_name ="XLip_0";
% control step used inside the controller in general different from time step for integration 
internal_dt = 0.05; 
% Parameters
infinity           = 10e6;
prm.h              = 0.8;%0.8;
prm.footSize_x     = 0.05;
prm.foot_to_foot_x = 0.0;        % desired foot to foot distance along x
prm.vref_x         = 0.1;        % desired com velocity

% LIP model
omega = sqrt(9.8/prm.h);

ch    = cosh(omega*internal_dt);
sh    = sinh(omega*internal_dt);
A_lip = [ch, sh/omega, 1-ch; omega*sh, ch, -omega*sh; 0, 0, 1];
B_lip = [internal_dt-sh/omega; 1-ch; internal_dt];

% Dummy states for foot-to-foot distance and reference velocity
A_dummy = 1;

% Full system
A_cont = blkdiag(A_lip, A_dummy);
B_cont = blkdiag(B_lip);
B_cont(end+1,end) = 0; % additional empty inputs for the dummy states


% System outputs
C_cont= [0, 1, 0,-1];  % com velocity to vref
 

% with this flag i tell the MPC constructor if the matrix has already been discretized or not      
discretized = true;
% with this i require to do the feedback linearization (by default is false)
feedback_lin  = false;
%% Initial state
init_state = [ 0; 0; 0;   % com position, com velocity and zmp position
               prm.vref_x];    % foot-to-foot distance and vRef 
          
%% Bounds (here the state bounds change for each walking phase)
maxOutputL = [infinity];
maxOutputR = [infinity];
boundsOutput.max  = [maxOutputL,maxOutputR];
boundsOutput.min  = [];
boundsInput.max   = [infinity; infinity; infinity];
% here max_output is empty because here we are going to use mutable bounds
          
B_Out.max  = [];
B_Out.min  = [];
B_In.max   = [];       
B_In.min   = [];           
          
%% gains
state_gain   = [100, 0, 0, 0, 0, 0];    % penalty error on the state
control_cost = [1,1,1]; 
%% predictive windows (it is useful for mutable constraints)
N            = 30;
%% here i define if the model is fixed or ltv
type         = "fixed"; 

%% creating data structure for mutable constraints (one for each state)
pattern_1 = [ones(N/2,1); zeros(N/2,1)];
pattern_2 = [zeros(N/2,1);ones(N/2,1)];
foot_pattern = [pattern_1,pattern_2];

mutable_constr.N_state           = 2;
mutable_constr.const_pattern     = foot_pattern;
mutable_constr.boundsOutput      = boundsOutput;
mutable_constr.boundsInput       = boundsInput;

mutable_constr.g    = false;
mutable_constr.w    = true;
mutable_constr.s    = false;

%% function list
function_list.propagationModel = "std";
function_list.costFunc         = "std";      
function_list.constrW          = "walking";      
function_list.constrG          = "std";    
function_list.constrS          = "std";  
