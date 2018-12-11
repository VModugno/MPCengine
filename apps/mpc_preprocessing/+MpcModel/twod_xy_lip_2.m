%% model short description
% 2d lip with both x and y direction plus the simple model for the feet
% movement both in x and y.
% in this model we employ the classic discretization (euler) for the lip model 
% and it is built in a way that we need to employ a regulator MPC

%% 2dxylip (this system is already discretized)
%% Model 
% name of the enviroment which the current model represents
env_name ="XYLip_2";
% delta is sym here because the system is already discretized
internal_dt = 0.05;
% Parameters
h              = 0.4845;%0.8;
infinity       = 10e6;
footSize_x     = 0.05;
footSize_y     = 0.03;
foot_to_foot_x = 0.0;         % desired foot to foot distance along x
foot_to_foot_y = -0.17;%-0.2  % desired foot to foot distance along x
ref_vel_x      = 0.1;         % desired com velocity
ref_vel_y      = 0;
max_f_to_f     = 0.05;        % bounds  

% LIP model
omega = sqrt(9.8/h);


A_lip = [0 1 0; omega^2 0 -omega^2;0 0 0];
B_lip = [0; 0; 1];

% discretization
A_lip = eye(3) + internal_dt*A_lip;
B_lip = internal_dt*B_lip;


% Foot model
A_foot = eye(2) + internal_dt*[0, 1; 0, 0];
B_foot = internal_dt*[0; 1];

% Dummy states for foot-to-foot distance and reference velocity
A_dummy = 1;

% Full system
A_x = blkdiag(A_lip, A_foot, A_foot, A_dummy, A_dummy);
A_y = blkdiag(A_lip, A_foot, A_foot, A_dummy, A_dummy);
B = blkdiag(B_lip, B_foot, B_foot);
B(end+2,end) = 0; % additional empty inputs for the dummy states
A = blkdiag(A_x, A_y);
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
% with this i require to do the feedback linearization (by default is false)
feedback_lin  = false;
%% Initial state
init_state = [ 0; 0; 0;   % com position, com velocity and zmp position
               0; 0;      % left foot position and velocity
               0; 0;      % right foot position and velocity
               foot_to_foot_x; ref_vel_x;    % foot-to-foot distance and vRef 
               0; 0; 0;   % com position, com velocity and zmp position
               0; 0;      % left foot position and velocity
             -0.2; 0;    % right foot position and velocity
              foot_to_foot_y; ref_vel_y]; % foot-to-foot distance and vRef   
          
%% Bounds (here the state bounds change for each walking phase)
maxOutputL = [infinity; 0; infinity; footSize_x; infinity; max_f_to_f;
              infinity; 0; infinity; footSize_y; infinity; max_f_to_f];
maxOutputR = [infinity; infinity; 0; infinity; footSize_x; max_f_to_f;
              infinity; infinity; 0; infinity; footSize_y; max_f_to_f];
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


