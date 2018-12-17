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
%% logging data for comparison with results obtained with c++ (it works only if start_simulation = true)
logging          = false;
%% MPC Model
model_name       = "twod_xy_lip_0"; 

%% experiment time structure (for the environment different from the internal time)
ft         = 5;       % 20 50 
delta_t    = 0.05;    % 0.1 0.01 (to the env class)
t          = 0:delta_t:ft;

%% MPC parameters
% cpp solver to use 
solver       = "QPoases";
% tracker or regulator
control_mode = "regulator"; 

%% regulator testing
if(strcmp(control_mode,"regulator")) 
    %% system -------------------------------------------------------------
    str        = "MpcModel." + model_name + ".m";
    run(str);
    %% environment for regulator ------------------------------------------
    reward     = @(x,u)(norm(x));
    env_call   = "Env."+ env_name + "(init_state,delta_t,reward,prm)";
    env        = eval(env_call);
    %% MPC ----------------------------------------------------------------    
    % regulator 
    controller = MpcGen.genMpcRegulator(A_cont,B_cont,C_cont,maxInput,maxOutput,internal_dt,N,state_gain,control_cost,...
                                        type,solver,generate_func,discretized,mutable_constr,function_list); 
elseif(strcmp(control_mode,"tracker"))
    %% system -------------------------------------------------------------
    str = "MpcModel." + model_name + ".m";
    run(str);
    %% enviroment for tracker ---------------------------------------------
    reward     = @(x,u)(norm(x));  %(TODO reward class)
    env_call   = "Env."+ env_name + "(init_state,delta_t,reward,prm)";
    env        = eval(env_call);
    %% reference ----------------------------------------------------------
    % x_des_model shoudl be define in the mpcMdel file
    x_des = x_des_model;
    
    %% MPC ----------------------------------------------------------------
    % tracker
    controller   = MpcGen.genMpcTracker(A_cont,B_cont,C_cont,maxInput,maxOutput,internal_dt,N,state_gain,control_cost,...
                                        type,solver,generate_func,discretized,mutable_constr);
    %% testing MPC tracking on the environment 
    
    % preparing the reference for testing
    total_ref = reshape(x_des,length(x_des)*controller.q,1);
    % extend ref by N to avoid the preceeding horizon to exceed the
    % final time
    last_ref = x_des(:,end);
    for i=1:N
        total_ref = [total_ref;last_ref];
    end
end

if(visualization) 
    env.Render();
end
if(start_simulation)
    cur_x           = init_state;
    all_states(:,1) = cur_x;
    for i=1:length(t)-1
        if(visualization)   
            env.UpdateRender(cur_x);  
        end
        %% mpc controller  
        if(strcmp(control_mode,"regulator"))
            tau = controller.ComputeControl(cur_x);          
        elseif(strcmp(control_mode,"tracker"))
            tau = controller.ComputeControl(cur_x,total_ref((i-1)*controller.q + 1:(i-1)*controller.q + controller.N*controller.q));
        end
        %% feedback linearization when required
        if(feedback_lin)
        %% only for test with MPC tracking with 2r robot for now
            dyn_comp   = env.GetDynamicalComponents(cur_x);
            tau        = dyn_comp.M*tau + dyn_comp.S*cur_x(3:4,1) + dyn_comp.g;
        end
         
        % update env
        [new_state]= env.Step(tau);
        % update variables
        cur_x             = new_state;
        all_states(:,i+1) = cur_x;
        all_action(:,i)   = tau;
    end
end


%% generating function (do not change this part)

if(generate_func)
    controller.GenFunctions();
    % function to create file with parameters
    controller.GenParametersFile();
    env.GenEnvParametersFile(controller.basepath,ft);
end
if(logging && start_simulation)
    str           = which('readme_log_mpc.txt');
    path          = fileparts(str);
    full_path     = strcat(path,'/trajectories_from_matlab.mat');
    % i need to transpose because the data from c are ordered as columns
    all_states_gt = all_states';
    all_action_gt = all_action';
    save(full_path,'all_states_gt','all_action_gt');
end
