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

%% tracking or regulator

control_mode    = "tracker"; % tracker, regulator

%% regulator testing
if(strcmp(control_mode,"regulator"))
    %cart pole parameters (to the env class) 
    mCart    = 0.1;
    mPend    = 0.1;
    L        = 0.5;
    g        = 9.8;
    xCartMax = 5;
    % Double integrator
    A_cont = [0 1 0 0; 0 0 -mPend*g/mCart 0; 0 0 0 1; 0 0 (mCart+mPend)*g/(L*mCart) 0];
    B_cont = [0; 1/mCart; 0; -1/(L*mCart)];
    C_cont = [1 0 0 0;
              0 0 1 0;
              0 0 0 1];
    maxOutput    = [10;100;100];
    maxInput     = [100];
    delta_t      = 0.1;     % (to the env class)
    N            = 30;       % prediction window
    state_gain   = 1000;    % penalty error on the state
    control_cost = 1;
    type         = "fixed"; 
    solver       = "QPoases";
    % regulator 
    controller = MpcGen.genMpcRegulator(A_cont,B_cont,C_cont,maxInput,maxOutput,delta_t,N,state_gain,control_cost,type,solver,generate_func);
    %% environment regulator
    ft        = 50;   
    t          = 0:delta_t:ft;
    init_state = [0; 0; pi/8; 0];
    reward     = @(x,u)(norm(x));
    % environment
    env        = Env.CartPole(init_state,delta_t,reward);
elseif(strcmp(control_mode,"tracker"))
    %% enviroment for tracker
    % time structure of the experiment-----------------------------------------
    ft         = 50; 
    delta_t    = 0.01;  
    t          = 0:delta_t:ft;
    % reference (TODO reference class)-----------------------------------------
    % sin traj
    q1des_t = pi/2*sin(t);
    q2des_t = pi/3*cos(t);

    dq1des_t = diff(q1des_t)/delta_t;
    dq1des_t = [dq1des_t , dq1des_t(end)];
    dq2des_t = diff(q2des_t)/delta_t;
    dq2des_t = [dq2des_t , dq2des_t(end)];

    x_des    = [q1des_t;q2des_t;dq1des_t;dq2des_t];


    % system ------------------------------------------------------------------
    A_cont       = [0 0 1 0 ; 0 0 0 1 ; 0 0 0 0; 0 0 0 0];
    B_cont       = [0 0; 0 0; 1 0; 0 1];
    C_cont       = eye(4);%[1 0 0 0; 0 1 0 0];
    maxOutput    = [10;10;10;10];
    maxInput     = [20;20];
    N            = 20;     % prediction window
    state_gain   = 100;     % penalty error on the state
    control_cost = 1;  
    type         = "fixed"; 
    solver       = "QPoases";

    controller   = MpcGen.genMpcTracker(A_cont,B_cont,C_cont,maxInput,maxOutput,delta_t,N,state_gain,control_cost,type,solver,generate_func);
    %% testing MPC tracking on the environment 
    init_state = [pi; pi/2; 0; 0];
    reward = @(x,u)(norm(x));  %(TODO reward class)
    env = Env.TwoRRobot(init_state,delta_t,reward,'ConfigFile','robot_payload');

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
            tau = ComputeControl(obj,cur_x);          
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
    end
end


%% generating function (do not change this part)

if(generate_func)
    controller.GenFunctions();
    % function to create file with parameters
    controller.GenParametersFile();
    env.GenEnvParametersFile(controller.basepath,ft);
end
