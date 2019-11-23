clear variables
close all
clc


%% activate or deactivate function generation
generate_func    = false;
%%
%% simulate the mpc 
start_simulation = true;
%% rotation variables
% do not change these parameters!!!
angle_index_selector = 0;   % for keeping track of the position on the angle sequence (i will exploit the N parameters)
trigger_update_index = 1;   % for udapting the sequence only at the frequence of the controller (internal_dt)
%%
%% activate or deactivate visualization
visualization    = true;
%%
%% logging data for comparison with results obtained with c++ (it works only if start_simulation = true)
logging          = true;
%%
%% debugging
debugging        = false;
%%
%% MPC Model
model_name       = "twod_xy_lip_no_foot_model_automatic_footstep_with_rotation";
%%
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
%% angle sequences generation
% i pregenerate in advance all the angles just for testing purposes
% IMPORTANT! THE ANGLES SEQUENCE HAS TO BE SYNCRONIZED WITH
% footstep_pattern
if(prm.active_rotation)
    step_of_rotation = round(prm.duration_of_rotation/internal_dt);
    step_counter   = 1;
    cur_theta      = prm.theta_0;
    last_cur_theta = prm.theta_0;
    for ijk = 1:step_of_rotation
        
        cur_theta = cur_theta +  prm.angular_velocity*internal_dt;
        
        if(step_counter<N/2)
            full_angles_sequence(ijk,1)=last_cur_theta;
            step_counter = step_counter + 1;
        else
            last_cur_theta = cur_theta;
            full_angles_sequence(ijk,1) = last_cur_theta;
            step_counter = 1;
        end
    end
    % i need it in order to keep the last angle fixed
    last_angle = full_angles_sequence(end,1);
    % initialize first sequence 
    angles_sequence = full_angles_sequence(angle_index_selector+1:angle_index_selector + N,1);
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
            %% this is a code to simulate a trajectory generator block
            if(prm.active_rotation)
                waiting_steps = internal_dt/ext_dt;
                % integrate omega (it has to be a column vector)
                % update only at each internal timestep
                if(trigger_update_index >= waiting_steps)
                    % here i update this index to pick the next element in full_angles_sequence
                    % that is not yet contained in angles_sequence
                    angle_index_selector = angle_index_selector + 1;
                    angles_sequence = angles_sequence(2:end,1);
                    if(angle_index_selector + N < length(full_angles_sequence))
                        angles_sequence(end+1,1) = full_angles_sequence(angle_index_selector + N);
                    else
                        angles_sequence(end+1,1) = last_angle;
                    end
                    % restart trigger update
                    trigger_update_index = 0;
                end
                % update index to syncronize the update
                trigger_update_index = trigger_update_index + 1;
                tau = controller.ComputeControl(cur_x,angles_sequence); 
            else
                % controller without rotation
                tau = controller.ComputeControl(cur_x);  
            end
            
            
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
        % for visualization purpose i pass this angle to the environment 
        % only works with an eviroment that accepts rotation
        if(prm.active_rotation)
            if(prm.fixed_direction)
                x_des  = [cos(prm.theta_0)*prm.traslation_velocity  sin(prm.theta_0)*prm.traslation_velocity];
            else
                x_des  = [cos(angles_sequence(1))*prm.traslation_velocity  sin(angles_sequence(1))*prm.traslation_velocity];
            end
            env.cur_angle = angles_sequence(1);
            % setDes has to be called before env.step 
            % here the update is at the integration frequency
            env.SetDes(x_des);
            % by passing x_des to env i will trigger the update of the
            % desired variables in the system
        end
        % update env
        [new_state]= env.Step(tau);
        % update variables 
        cur_x             = new_state;
        % for logging (TODO we need to move this stuff inside step)
        all_states(:,i+1) = cur_x;
        new_tau           = env.ReshapeAction(tau);
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
