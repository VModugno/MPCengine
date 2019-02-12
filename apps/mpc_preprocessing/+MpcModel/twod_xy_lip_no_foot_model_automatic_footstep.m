%% model short description
% 2d lip with both x and y direction without the model for the feet 
% but with the automatic footstep planning in place
% in this model we employ a specific discretization for the lip model 
% and it is built in a way that we need to employ a regulator MPC (? im not sure still)
% for now we categorize this model under the "custom" class but in the future
% we could extend this case to be an actual separate problem class (as much as fixed and ltv)
% this could represents a system with scheduled change in structure

%% for now I'm considering everything in a world reference frame but i could reconsider this choice 


%% 2dxylip (this system is already discretized)
%% Model 
% name of the enviroment which the current model represents
env_name ="XYLip_0";
% control step used inside the controller in general different from time step for integration 
internal_dt = 0.05; 
% Parameters
infinity           = 10e6;
prm.h              = 0.8;%0.8;
prm.footSize_x     = 0.05;
prm.footSize_y     = 0.03;
prm.foot_to_foot_x = 0.0;        % desired foot to foot distance along x
prm.foot_to_foot_y = -0.2;%-0.2  % desired foot to foot distance along x
prm.vref_x         = 0.1;        % desired com velocity
prm.vref_y         = 0;
max_f_to_f         = 0.05;       % bounds  

% LIP model
omega = sqrt(9.8/prm.h);

ch    = cosh(omega*internal_dt);
sh    = sinh(omega*internal_dt);
A_lip = [ch, sh/omega, 1-ch; omega*sh, ch, -omega*sh; 0, 0, 1];
B_lip = [internal_dt-sh/omega; 1-ch; internal_dt];

% here we consider a discretized system for the footstep of this type:
% (x_f^{k+1} - x_f^{k})/dt = u_2 for the beggining of the single support
% (x_f^{k+1} - x_f^{k})/dt = 0   otherwise
% footstep propagation model already discretized I + dt*A
A_f   = [1];

% Full system (two versions) (number of states = 8, number of inputs = 4 or 2 depending on the model)
A_x   = blkdiag(A_lip, A_f);
A_y   = blkdiag(A_lip, A_f);
B_x_1 = blkdiag(B_lip, A_f);
B_x_2 = blkdiag(B_lip);
B_y_1 = blkdiag(B_lip, A_f);
B_y_2 = blkdiag(B_lip);
B1    = blkdiag(B_x_1,B_y_1);
B2    = blkdiag(B_x_2,B_y_2);

% for this case A_cont B_cont and C_cont are a sequence of matrices (cell vectors) and we need to
% define a mask for each of them
A_cont = {blkdiag(A_x, A_y)};
B_cont = {B1,B2};

% System outputs (C_x = C_y)
C_x_cost = eye(4);
C_x_mes  = [0 0 1 -1];
% for now i put everything inside C_cont even if 
C_cont = {blkdiag(C_x_cost,C_x_cost);blkdiag(C_x, C_x)};

% with this flag i tell the MPC constructor if the matrix has already been discretized or not      
discretized   = true;
% with this i require to do the feedback linearization (by default is false)
feedback_lin  = false;
%% Initial state
init_state = [ 0; 0; 0; 0;   % sagittal axis (x coordinate) com position, com velocity, zmp position and initial footstep 
               0; 0; 0; 0];  % coronal  axis (y coordinate) foot position and velocity, zmp position and initial footstep
             
          
%% Bounds for mutable constraints (here the state bounds change for each walking phase)
maxOutputL = [infinity; 0; infinity; prm.footSize_x; infinity; max_f_to_f;
              infinity; 0; infinity; prm.footSize_y; infinity; max_f_to_f];
maxOutputR = [infinity; infinity; 0; infinity; prm.footSize_x; max_f_to_f;
              infinity; infinity; 0; infinity; prm.footSize_y; max_f_to_f];
boundsOutput.max  = [maxOutputL,maxOutputR];
boundsOutput.min  = [];
boundsInput.max   = [infinity; infinity; infinity;
                     infinity; infinity; infinity];
% here max_output is empty because here we are going to use mutable bounds
          
B_Out.max  = [];
B_Out.min  = [];
B_In.max   = [];       
B_In.min   = [];           
          
%% gains
state_gain   = [100, 0, 0, 0, 0, 0,100, 0, 0, 0, 0, 0];    % penalty error on the state
control_cost = [1,1,1,1,1,1]; 
%% predictive windows (it is useful for mutable constraints)
N            = 30;
%% here i define if the model is fixed or ltv or custom (with custom we can admit any kind of construction)
type         = "custom"; 

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
