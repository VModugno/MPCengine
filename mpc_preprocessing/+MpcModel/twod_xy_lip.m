%% 2dxylip (this system is already discretized)
%% Model 
% name of the enviroment which the current model represents
env_name ="XYLip";
% Parameters
syms delta; 
omega          = sqrt(9.8/0.8);
infinity       = 10e6;
footSize_x     = 0.05;
footSize_y     = 0.03;
foot_to_foot   = 1;
ref_vel        = 1;

% LIP model
ch    = cosh(omega*delta);
sh    = sinh(omega*delta);
A_lip = [ch, sh/omega, 1-ch; omega*sh, ch, -omega*sh; 0, 0, 1];
B_lip = [delta-sh/omega; 1-ch; delta];

% Foot model
A_foot = eye(2) + delta*[0, 1; 0, 0];
B_foot = delta*[0; 1];

% Dummy states for foot-to-foot distance and reference velocity
A_distance = foot_to_foot;
A_vref     = ref_vel;

% Full system
A = blkdiag(A_lip, A_foot, A_foot, A_distance, A_vref);
B = blkdiag(B_lip, B_foot, B_foot);
B(end+2,end) = 0; % additional empty inputs for the dummy states
A = blkdiag(A, A);
B = blkdiag(B, B);

% System outputs
C  = [0, 1, 0, 0, 0, 0, 0, 0,-1;  % com velocity to vref 
      0, 0, 0, 0, 1, 0, 0, 0, 0;  % left foot velocity
      0, 0, 0, 0, 0, 0, 1, 0, 0;  % right foot velocity
      0, 0, 1,-1, 0, 0, 0, 0, 0;  % zmp from left foot
      0, 0, 1, 0, 0,-1, 0, 0, 0;  % zmp from right foot
      0, 0, 0, 1, 0,-1, 0, 1, 0]; % foot to foot
 C = blkdiag(C, C);

% with this flag i tell the MPC constructor if the matrix has already been discretized or not      
discretized = true;
%% Initial state
init_state = [ 0; 0; 0;   % com position, com velocity and zmp position
               0; 0;      % left foot position and velocity
               0; 0;      % right foot position and velocity
               0; 0.1;    % foot-to-foot distance and vRef
               0; 0; 0;   % com position, com velocity and zmp position
               0; 0;      % left foot position and velocity
             -0.2; 0;    % right foot position and velocity
             -0.2; 0]; % foot-to-foot distance and vRef 
 
%% Bounds (here the state bounds change for each walking phase)
maxOutputL = [infinity; 0; infinity; footSize_x; infinity; 0.05;
              infinity; 0; infinity; footSize_y; infinity; 0.05];
maxOutputR = [infinity; infinity; 0; infinity; footSize_x; 0.05;
              infinity; infinity; 0; infinity; footSize_y; 0.05];
bounds     = [maxOutputL,maxOutputR];
% here max_output is empty because here we are going to use mutable bounds
maxOutput  = [];
maxInput   = [infinity; infinity; infinity;
              infinity; infinity; infinity];         
%% 2 r robot gains
state_gain   = [100, 0, 0, 0, 0, 0,100, 0, 0, 0, 0, 0];    % penalty error on the state
control_cost = [1,1,1,1,1,1]; 
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
mutable_constr.bounds            = bounds;


