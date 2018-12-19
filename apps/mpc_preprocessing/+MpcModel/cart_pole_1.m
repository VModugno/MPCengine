%% model short description

% this model represent a simple cart pole problem linearized around the up
% up position. it is mainly used for testing the mpc controller for
% regulation problem. it can be used without feedback lin if the initial
% position is around a up up position. we do not ahve implemented andy
% fucntion for feedback linearization so far, this version is conceived for
% an ltv MPC implementation


%% cart pole model 
% name of the enviroment which the current model represents
env_name ="CartPole";
% control step used inside the controller in general different from time step for integration 
internal_dt = 0.05; 
%cart pole parameters (to the env class) (TODO add read parameters from file)
prm.mCart    = 0.1;
prm.mPend    = 0.1;
prm.L        = 0.5;
g            = 9.8;
xCartMax     = 5;

% for ltv I need to define the parametrized linearized model using the
% symbolic toolbox

n = 4;
m = 1;

x = sym('x',[n,1],'real');
u = sym('u',[m,1],'real');

mes_acc   = [(u + prm.mPend*sin(x(3))*(prm.L*x(4)^2-g*cos(x(3))))/(prm.mCart+prm.mPend*sin(x(3))^2);...
            (-u*cos(x(3)) - prm.mPend*prm.L*x(4)^2*sin(x(3))*cos(x(3)) + (prm.mPend+prm.mCart)*g*sin(x(3)))/(prm.L*(prm.mCart+prm.mPend*sin(x(3))^2))];

f         = [x(2); ...
            mes_acc(1);...
            x(4);...
            mes_acc(2)];

A_cont = jacobian(f, x);
B_cont = jacobian(f, u);
C_cont = [1 0 0 0;
          0 0 1 0;
          0 0 0 1];
% with this flag i tell the MPC constructor if the matrix has already been discretized or not      
discretized   = false;
% with this i require to do the feedback linearization (by default is false)
feedback_lin  = false;
%% init state
init_state = [0; 0; pi/8; 0];
%% cart pole state and control bounds  
B_Out.max    = [10;100;100];
B_Out.min    = [];
B_In.max     = [100];
B_In.min     = [];
%% gains
state_gain   = 100;    % penalty error on the state
control_cost = 1; 
%% predictive windows (it is useful for mutable constraints)
N            = 10;  
%% here i define if the model is fixed or ltv
type         = "ltv"; % if the model is type ltv i need to specify the system matrix already using symbolic varialbes
%% here i define if the model is with variable constraints or not
mutable_constr = [];
%% function list
function_list.propagationModel = "std";
function_list.costFunc         = "std";      
function_list.constrW          = "std";      
function_list.constrG          = "std";    
function_list.constrS          = "std"; 

%% desired trajectories
xc_des        = 0*ones(size(t));
theta_des     = 0*ones(size(t));
theta_dot_des = 0*ones(size(t));


x_des_model    = [xc_des;theta_des;theta_dot_des];