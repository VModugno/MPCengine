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
env_name ="XYLip_simplified_feet";
% control step used inside the controller in general different from time step for integration 
internal_dt = 0.05; 
% Parameters
infinity                          = 10e6;
prm.h                             = 0.26;%0.8;
prm.footSize_x                    = 0.05;
prm.footSize_y                    = 0.03;
prm.foot_to_foot_x                = 0.0;        % desired foot to foot distance along x
prm.foot_to_foot_y                = -0.2;%-0.2  % desired foot to foot distance along x
prm.vref_x                        = 0.1;        % desired com velocity
prm.vref_y                        = 0;
max_f_to_f                        = 0.05;       % bounds  
prm.footstep_constraints_x        = 0.1;
prm.inner_footstep_constraints_y  = 0.20;
prm.outer_footstep_constraints_y  = 0.25; 
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
A_f   =  1; 
B_f   =  1;
% reference propagation model 
A_ref =  1;
B_ref =  0;
% Full system (two versions) (number of states = 8, number of inputs = 4 or 2 depending on the model)
A_x   = blkdiag(A_lip, A_f, A_ref);
A_y   = blkdiag(A_lip, A_f, A_ref);
B_x_1 = blkdiag(B_lip, B_f);
B_x_1 = [B_x_1;0,0];
B_y_1 = blkdiag(B_lip, B_f);
B_y_1 = [B_y_1;0,0];
B_x_2 = [B_lip;0;0];
B_y_2 = [B_lip;0;0];
A1    = blkdiag(A_x, A_y);
A2    = blkdiag(A_x, A_y);
B1    = blkdiag(B_x_1,B_y_1);
B2    = blkdiag(B_x_2,B_y_2);

% for this case A_cont B_cont and C_cont are a sequence of matrices (cell vectors) and we need to
% define a mask for each of them
A_cont = {A1,A2};
B_cont = {B1,B2};

% System outputs (C_x = C_y)
C_x_objective    = [0 1 0 0 -1];
C_x_constraints  = [0 0 1 -1 0];
C_cont_obj       = {blkdiag(C_x_objective,C_x_objective),blkdiag(C_x_objective,C_x_objective)};
% for now i put everything inside C_cont even if 
C_cont_constr    = {blkdiag(C_x_constraints,C_x_constraints);blkdiag(C_x_constraints,C_x_constraints)};
%% predictive windows (it is useful for mutable constraints)
N                = 12;
%% here i define if the model is fixed or ltv or statemachine custom (with custom we can admit any kind of construction)
type             = "statemachine";
%% here we introduce a state machine pattern (one for each model)

% state_pattern               = [2*ones(N/2-1,1);1;2*ones(N/2-1,1);1];
state_pattern               = [2*ones(N/2-1,1);1;2*ones(N/2-1,1);1];
state_machine.state_pattern = state_pattern;
% i need this to reset the pattern
state_machine.reset         = state_pattern;
state_machine.n_of_models   = 2;

% with this flag i tell the MPC constructor if the matrix has already been discretized or not      
discretized   = true;
% with this i require to do the feedback linearization (by default is false)
feedback_lin  = false;
%% Initial state
init_state = [  0;    0; 0; 0  ;  0.1;  % sagittal axis (x coordinate) com position, com velocity, zmp position, initial footstep and vrefx
                0;    0; 0; 0.1;  0];  % coronal  axis (y coordinate) foot position and velocity, zmp position, initial footstep and vrefy
             
          
%% here we consider the case with mutable bounds induced by  
 % Bounds from state machine and mutable constraints
 % the entry of the cell array for the bounds represents each model in the
 % state machine while the columns of each item represents the mutable
 % constraints associated with each foot
 

maxOutputL = [prm.footSize_x; prm.footSize_y];
maxOutputR = [prm.footSize_x; prm.footSize_y];
boundsOutput.max{1} = [maxOutputL,maxOutputR];
boundsOutput.max{2} = [maxOutputL,maxOutputR];
boundsOutput.min{1} = [-maxOutputL,-maxOutputR];
boundsOutput.min{2} = [-maxOutputL,-maxOutputR];


maxInputL          = [infinity; prm.footstep_constraints_x; infinity; prm.outer_footstep_constraints_y];
maxInputR          = [infinity; prm.footstep_constraints_x; infinity; -prm.inner_footstep_constraints_y];
maxInput_allFeet   = [infinity; infinity];
boundsInput.max{1} = [maxInputL,maxInputR];
boundsInput.max{2} = [maxInput_allFeet,maxInput_allFeet];

minInputL          = [-infinity; -prm.footstep_constraints_x; -infinity;  prm.inner_footstep_constraints_y];
minInputR          = [-infinity; -prm.footstep_constraints_x; -infinity; -prm.outer_footstep_constraints_y];
minInput_allFeet   = [-infinity; -infinity];
boundsInput.min{1} = [minInputL,minInputR];
boundsInput.min{2} = [minInput_allFeet,minInput_allFeet];

                     
                 
% here max_output is empty because here we are going to use mutable bounds
          
B_Out.max  = [];
B_Out.min  = [];
B_In.max   = [];       
B_In.min   = [];           
          
%% gains
state_gain   = {[100,100],[100,100]};    % penalty error on the state
control_cost = {[1,0,1,0],[1,1]}; 

%% creating data structure for mutable constraints (one for each state)
% in the footstep pattern we associate 1 to left and 2 to right
% footstep_pattern = [ones(N/2,1);2*ones(N/2,1)];
footstep_pattern = [ones(N/2-1,1);2*ones(N/2,1);3];
pattern_1 = mod(footstep_pattern,2)==1;
pattern_2 = mod(footstep_pattern,2)==0;
foot_pattern = [pattern_1,pattern_2];

mutable_constr.N_state           = 2;
mutable_constr.footstep_pattern  = footstep_pattern;
mutable_constr.reset_footstep    = footstep_pattern;
mutable_constr.const_pattern     = foot_pattern;
mutable_constr.reset_pattern     = foot_pattern;
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
