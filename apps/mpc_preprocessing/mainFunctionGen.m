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
visualization    = true;
%%
%% logging data for comparison with results obtained with c++ (it works only if start_simulation = true)
logging          = true;
%% debugging
debugging        = false;
%% MPC Model
model_name       = "twod_xy_lip_no_foot_model_automatic_footstep"; 

%% experiment time structure (for the environment different from the internal time)
ft          = 10;       % final time 20 50 
ext_dt      = 0.01;     % 0.1 0.01 (to the env class)
t           = 0:ext_dt:ft;

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
    env_call   = "Env."+ env_name + "(init_state,ext_dt,reward,prm)";
    env        = eval(env_call);
    
    %% DEBUG
    env_vis_debug    = eval(env_call);
    
    %% MPC ----------------------------------------------------------------    
    % regulator 
    controller = MpcGen.genMpcRegulator(A_cont,B_cont,C_cont_obj,C_cont_constr,B_In,B_Out,internal_dt,ext_dt,N,state_gain,control_cost,...
                                        type,solver,generate_func,discretized,mutable_constr,state_machine,non_standard_prediction_win,function_list); 
elseif(strcmp(control_mode,"tracker"))
    %% system -------------------------------------------------------------
    str = "MpcModel." + model_name + ".m";
    run(str);
    %% enviroment for tracker ---------------------------------------------
    reward     = @(x,u)(norm(x));  %(TODO reward class)
    env_call   = "Env."+ env_name + "(init_state,delta_t,reward,prm)";
    env        = eval(env_call);
    %% reference ----------------------------------------------------------
    % TODO x_des_model shoudl be define in the mpcModel file
    x_des = x_des_model;
    
    %% MPC ----------------------------------------------------------------
    % tracker
    controller   = MpcGen.genMpcTracker(A_cont,B_cont,C_cont,B_In,B_Out,internal_dt,N,state_gain,control_cost,...
                                        type,solver,generate_func,discretized,mutable_constr,function_list);
    %% testing MPC tracking on the environment 
    
    % preparing the reference for testing
    total_ref = reshape(x_des,length(x_des)*controller.q,1);
    % extend ref by N to avoid the preceeding horizon to exceed the
    % final time
    last_ref = x_des(:,end);
    for i=1:N
        total_ref = [total_ref;last_ref]; %#ok<AGROW>
    end
end

if(visualization) 
    env.Render();
end
if(debugging)
    env_vis_debug.Render();
end
if(start_simulation)
    cur_x           = init_state;
    all_states(:,1) = cur_x;
    for i=1:length(t)-1
%         if(visualization)   
%             env.UpdateRender(cur_x);  
%         end
        %% mpc controller  
        if(strcmp(control_mode,"regulator"))
            tau = controller.ComputeControl(cur_x);  
            
            %% DEBUG
            if(debugging)
                env_vis_debug.SetState(cur_x)
                cur_x_debug = cur_x;
                for jj = 1:length(controller.u_star_debug) 
                    env_vis_debug.UpdateRender(cur_x_debug);  
                    cur_x_debug = env_vis_debug.Step(controller.u_star_debug{jj});
                end
                disp('after debug')
            end
        elseif(strcmp(control_mode,"tracker"))
            %tau = controller.ComputeControl(cur_x,total_ref((i-1)*controller.q + 1:(i-1)*controller.q + controller.N*controller.q));
            tau = controller.ComputeControl(cur_x,total_ref((i-1)*controller.q + 1:(i-1)*controller.q + controller.N*controller.q + controller.N + 2));
        end
        %% feedback linearization when required
        if(feedback_lin)
        %% only for test with MPC tracking with 2r robot for now
            dyn_comp   = env.GetDynamicalComponents(cur_x);
            tau        = dyn_comp.M*tau + dyn_comp.S*cur_x(3:4,1) + dyn_comp.g;
        end
         
        % check if the trigger update is true
        env.ReadTriggerUp(controller.trigger_update);
        % update env
        [new_state]= env.Step(tau);
        % update variables 
        cur_x             = new_state;
        % for logging (TODO we need to move this stuff inside step)
        all_states(:,i+1) = cur_x;
        new_tau = env.ReshapeAction(tau);
        all_action(:,i)   = new_tau;
        
        cur_time = "t = " + num2str(t(i));
        disp(cur_time)
        disp(i)
        
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
