clear variables
close all
clc

%% activate or deactivate function generation
generate_func    = true;
%%
%% simulate the mpc 
start_simulation = false;
%%
%% activate or deactivate visualization
visualization    = false;
%%

% cart pole parameters (to the env class)
mCart = 0.1;
mPend = 0.1;
L = 0.5;
g = 9.8;
xCartMax = 5;
% Double integrator
A_cont = [0 1 0 0; 0 0 -mPend*g/mCart 0; 0 0 0 1; 0 0 (mCart+mPend)*g/(L*mCart) 0];
B_cont = [0; 1/mCart; 0; -1/(L*mCart)];
C_cont = [1 0 0 0;
          0 0 1 0;
          0 0 0 1];
maxOutput    = [10;100;100];
maxInput     = [20];
delta_t      = 0.1;     % (to the env class)
N            = 20;       % prediction window
state_gain   = 100;     % penalty error on the state
control_cost = 1;
type         = "fixed"; 
solver       = "QPoases";
% regulator 
controller = MpcGen.genMpcRegulator(A_cont,B_cont,C_cont,maxInput,maxOutput,delta_t,N,state_gain,control_cost,type,solver);

%% testing MPC on the environment 
ft        = 50;   
t          = 0:delta_t:ft;
init_state = [0; 0; pi/4; 0];
reward     = @(x,u)(norm(x));
% environment
env        = Env.CartPole(init_state,delta_t,reward);

if(visualization) 
    env.Render();
end
if(start_simulation)
    cur_x = init_state;
    all_states(:,1) = cur_x;
    for i=1:length(t)-1
        if(visualization)   
            env.UpdateRender(cur_x);  
        end
        % mpc 
        u          = controller.ComputeControl(cur_x);
        % update env
        [new_state]= env.Step(u);
        % udapte variables
        cur_x             = new_state;
        all_states(:,i+1) = cur_x;
    end
end


%% generating function (do not change this part)

if(generate_func)
    controller.GenFunctions();
    % function to create file with parameters
    controller.GenParametersFile();
    env.GenEnvParametersFile(controller.basepath,ft);
end
