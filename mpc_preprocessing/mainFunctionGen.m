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
%% logging data for comparison with results obtained with c++
logging          = true;
%% tracking or regulator
control_mode     = "regulator"; % tracker, regulator
%% MPC Model
model_name       = "twod_xy_lip_0";

%% experiment time structure
ft         = 5;       % 20 50 
delta_t    = 0.05;    % 0.1 0.01 (to the env class)
t          = 0:delta_t:ft;

%% regulator testing
if(strcmp(control_mode,"regulator")) 
    %% system -------------------------------------------------------------
    str        = "MpcModel." + model_name + ".m";
    run(str);
    % if the system is already discretized i need to substitute the
    % symbolical dt with its actual value
    if(discretized)
        A_cont = double(subs(A,delta_t));
        B_cont = double(subs(B,delta_t));
        C_cont = double(subs(C,delta_t));
    end
    %% environment for regulator ------------------------------------------
    reward     = @(x,u)(norm(x));
    env_call   = "Env."+ env_name + "(init_state,delta_t,reward)";
    env        = eval(env_call);
    %% MPC ----------------------------------------------------------------    
    % cpp solver to use  
    solver       = "QPoases";
    % regulator 
    controller = MpcGen.genMpcRegulator(A_cont,B_cont,C_cont,maxInput,maxOutput,delta_t,N,state_gain,control_cost,...
                                        type,solver,generate_func,discretized,mutable_constr); 
elseif(strcmp(control_mode,"tracker"))
    %% system -------------------------------------------------------------
    str = "MpcModel." + model_name + ".m";
    run(str);
    % if the system is already discretized i need to substitute the
    % symbolical dt with its actual value
    if(discretized)
        A_cont = double(subs(A,delta_t));
        B_cont = double(subs(B,delta_t));
        C_cont = double(subs(C,delta_t));
    end
    %% enviroment for tracker ---------------------------------------------
    reward     = @(x,u)(norm(x));  %(TODO reward class)
    env_call   = "Env."+ env_name + "(init_state,delta_t,reward)";
    env        = eval(env_call);
    %% reference (TODO reference class)------------------------------------
    % sin traj
    %q1des_t  = pi/2*sin(t);
    %q2des_t  = pi/3*cos(t);
    %dq1des_t = pi/2*cos(t);
    %dq2des_t = -pi/3*sin(t);
    
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
    f_to_f_y      = -0.2*ones(size(t));
   
    x_des    = [v_com_x_ref;v_foot_x_L;v_foot_x_R;zmp_foot_x_L;zmp_foot_x_R;f_to_f_x;...
                v_com_y_ref;v_foot_y_L;v_foot_y_R;zmp_foot_y_L;zmp_foot_y_R;f_to_f_x];
    
    %% MPC ----------------------------------------------------------------
    % cpp solver to use 
    solver       = "QPoases";
    % tracker
    controller   = MpcGen.genMpcTracker(A_cont,B_cont,C_cont,maxInput,maxOutput,delta_t,N,state_gain,control_cost,...
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
    cur_x = init_state;
    all_states(:,1) = cur_x;
    for i=1:length(t)-1
        if(visualization)   
            env.UpdateRender(cur_x);  
        end
        % mpc controller  
        if(strcmp(control_mode,"regulator"))
            tau = controller.ComputeControl(cur_x);          
        elseif(strcmp(control_mode,"tracker"))
            u          = controller.ComputeControl(cur_x,total_ref((i-1)*controller.q + 1:(i-1)*controller.q + controller.N*controller.q));
            %% only for test with MPC tracking with 2r robot (for this test the reference model for control is the same as the real system)
            dyn_comp   = env.GetDynamicalComponents(cur_x);
            tau        = dyn_comp.M*u + dyn_comp.S*cur_x(3:4,1) + dyn_comp.g;
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
