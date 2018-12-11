%% model short description
% 2d lip with both x and y direction plus the simple model for the feet
% movement both in x and y.
% in this model we employ a specific discretization for the lip model 
% and it is built in a way that we need to employ a tracking MPC


%% 2dxylip (this system is already discretized)
%% Model 
% name of the enviroment which the current model represents
env_name ="XYLip_1";
% control step used inside the controller in general different from time step for integration 
internal_dt = 0.05; 
% Parameters
h              = 0.8;
infinity       = 10e6;
footSize_x     = 0.05;
footSize_y     = 0.03;
%foot_to_foot_x = 0.0;        % desired foot to foot distance along x
%foot_to_foot_y = -0.2;       % desired foot to foot distance along x
%ref_vel_x      = 0.2;        % desired com velocity
%ref_vel_y      = 0;
max_f_to_f     = 0.21;        % bounds  

% LIP model
omega = sqrt(9.8/h);

ch    = cosh(omega*internal_dt);
sh    = sinh(omega*internal_dt);
A_lip = [ch, sh/omega, 1-ch; omega*sh, ch, -omega*sh; 0, 0, 1];
B_lip = [delta-sh/omega; 1-ch; internal_dt];
%A_lip = [0 1; omega^2 0];
%B_lip = [0; omega^2];

%A_lip = eye(2) + delta*A_lip;
%B_lip = delta*B_lip;

% Foot model
A_foot = eye(2) + internal_dt*[0, 1; 0, 0];
B_foot = internal_dt*[0; 1];

% Full system
A_x = blkdiag(A_lip, A_foot, A_foot);
A_y = blkdiag(A_lip, A_foot, A_foot);
B = blkdiag(B_lip, B_foot, B_foot);
A = blkdiag(A_x, A_y);
B = blkdiag(B, B);

% System outputs
C  = [0, 1, 0, 0, 0, 0, 0;  % com velocity 
      0, 0, 0, 0, 1, 0, 0;  % left foot velocity
      0, 0, 0, 0, 0, 0, 1;  % right foot velocity
      0, 0, 1,-1, 0, 0, 0;  % zmp from left foot
      0, 0, 1, 0, 0,-1, 0;  % zmp from right foot
      0, 0, 0, 1, 0,-1, 0]; % foot to foot
 C = blkdiag(C, C);

% with this flag i tell the MPC constructor if the matrix has already been discretized or not      
discretized = true;
% with this i require to do the feedback linearization (by default is false)
feedback_lin  = false;
%% Initial state
init_state = [ 0; 0; 0;   % com position, com velocity and zmp position
               0; 0;      % left foot position and velocity
               0; 0;      % right foot position and velocity
               0; 0; 0;   % com position, com velocity and zmp position
               0; 0;      % left foot position and velocity
            -0.2; 0];     % right foot position and velocity
         
          
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
%%gains
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

%% here i define the trjecotory for 
v_com_x_ref   = 0.1*ones(size(t));
v_foot_x_L    = 0*ones(size(t));
v_foot_x_R    = 0*ones(size(t));
zmp_foot_x_L  = 0*ones(size(t));
zmp_foot_x_R  = 0*ones(size(t));
f_to_f_x      = 0*ones(size(t));

v_com_y_ref   = 0*ones(size(t));
v_foot_y_L    = 0*ones(size(t));
v_foot_y_R    = 0*ones(size(t));
zmp_foot_y_L  = 0*ones(size(t));
zmp_foot_y_R  = 0*ones(size(t));
f_to_f_y      = -0.3*ones(size(t));
% you have to specify a x_des_model otherwise it is going to fail   
x_des_model    = [v_com_x_ref;v_foot_x_L;v_foot_x_R;zmp_foot_x_L;zmp_foot_x_R;f_to_f_x;...
                  v_com_y_ref;v_foot_y_L;v_foot_y_R;zmp_foot_y_L;zmp_foot_y_R;f_to_f_y];

