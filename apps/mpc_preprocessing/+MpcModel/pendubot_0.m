%% model short description

% this model represent a simple cart pole problem linearized around the up
% up position. it is mainly used for testing the mpc controller for
% regulation problem. it can be used without feedback lin if the initial
% position is around a up up position. we do not ahve implemented andy
% fucntion for feedback linearization so far, this version is conceived for
% an ltv MPC implementation


%% cart pole model 
% name of the enviroment which the current model represents
env_name ="Pendubot";
% control step used inside the controller in general different from time step for integration 
internal_dt = 0.05; 
%cart pole parameters (to the env class) (TODO add read parameters from file)
prm.m1 = 0.09;
prm.m2 = 0.08;
prm.l1 = 0.1492;
prm.l2 = 0.1905;
prm.lc1 = prm.l1/2;
prm.lc2 = prm.l2/2;
prm.I1 = (prm.m1*prm.l1^2)/12;
prm.I2 = (prm.m2*prm.l2^2)/12;
prm.Im = 4.74e-4;

% for ltv I need to define the parametrized linearized model using the
% symbolic toolbox

n = 4;
m = 1;

x = sym('x',[n,1],'real');
u = sym('u',[m,1],'real');


M = zeros(2,2);
M = sym(M);
M(1,1) = simplify(prm.m1*prm.lc1^2+prm.m2*(prm.l1^2+prm.lc2^2+2*prm.l1*prm.lc2*cos(x(2)))+prm.I1+prm.I2+prm.Im);
M(1,2) = simplify(prm.m2*(prm.lc2^2+prm.l1*prm.lc2*cos(x(2)))+prm.I2);
M(2,1) = M(1,2);
M(2,2) = prm.m2*prm.lc2^2+prm.I2;
C = zeros(2,1);
C = sym(C);
C(1,1) = simplify(-prm.m2*prm.l1*prm.lc2*sin(x(2))*x(4)^2 - 2*prm.m2*prm.l1*prm.lc2*sin(x(2))*x(3)*x(4));
C(2,1) = simplify(prm.m2*prm.l1*prm.lc2*sin(x(2))*x(3)^2);
phi = zeros(2,1);
phi = sym(phi);
g = 9.8;
phi(1,1) = simplify((prm.m1*prm.lc1+prm.m2*prm.l1)*g*cos(x(1)+pi/2)+prm.m2*prm.lc2*g*cos(x(1)+pi/2+x(2)));
phi(2,1) = simplify(prm.m2*prm.lc2*g*cos(x(1)+pi/2+x(2)));

mes_acc   = M\(u - C - phi);

f         = [x(3); ...
             x(4);...
             mes_acc(1);...
             mes_acc(2)];

A_cont = jacobian(f, x);
B_cont = jacobian(f, u);
C_cont = [1 0 0 0;
          0 1 0 0;
          0 0 1 0;
          0 0 0 1];
% with this flag i tell the MPC constructor if the matrix has already been discretized or not      
discretized   = false;
% with this i require to do the feedback linearization (by default is false)
feedback_lin  = false;
%% init state
init_state = [0; 0; 0; 0];
%% cart pole state and control bounds  
B_Out.max    = [10;100;100;100];
B_Out.min    = [];
B_In.max     = [10000];
B_In.min     = [];
%% gains
state_gain   = 100;    % penalty error on the state
control_cost = 1; 
%% predictive windows (it is useful for mutable constraints)
N            = 4;  
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
%xc_des        = 0*ones(size(t));
theta_dot_des = 0*ones(size(t));


x_des_model    = [xc_des;theta_des;xc_des;theta_dot_des];
%x_des_model    = [xc_des;theta_des;theta_dot_des];