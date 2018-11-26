clear variables 
close all 
clc



%% tracking or regulator
control_mode     = "tracker"; % tracker, regulator
%% activate or deactivate visualization
visualization    = false;


% open file inside @log
state_cpp  = load('state_from_mpc_cpp.mat','-ascii');
action_cpp = load('action_from_mpc_cpp.mat','-ascii');
load('trajectories_from_matlab.mat');

%verify that the sizes are the same and if is not true i shorten the longer vector 
if(length(state_cpp)~= length(all_states_gt))
    if(length(all_states_gt)>length(state_cpp))
        diff = length(all_states_gt) - length(state_cpp);
        for i=1:diff
            all_states_gt(end,:) = [];
        end
    else
        diff = length(state_cpp)-length(all_states_gt);
        for i=1:diff
            state_cpp(end,:) = [];
        end
    end
end
if(length(action_cpp)~= length(all_action_gt))
    if(length(all_action_gt)>length(action_cpp))
        diff = length(all_action_gt) - length(action_cpp);
        for i=1:diff
            all_action_gt(end,:) = [];
        end
    else
        diff = length(action_cpp)-length(all_action_gt);
        for i=1:diff
            action_cpp(end,:) = [];
        end
    end
end
state_average_error  = sum(sum((state_cpp - all_states_gt).^2,2))/length(all_states_gt);
action_average_error = sum(sum((action_cpp- all_action_gt).^2,2))/length(all_action_gt);

if(visualization)
    state = state_cpp;
    % % plot using the enviroment 
    % delta_t    = 0.1;
    % init_state = [0; 0; pi/10; 0];
    % reward     = @(x,u)(norm(x)); % dummy reward
    % env        = Env.CartPole(init_state,delta_t,reward);
    % 
    % env.Render();
    % 
    % for ii=1:size(state,1)
    %     
    %     env.UpdateRender(state(ii,:)');
    %     pause(30/1000)
    %     
    % end

    % plot using the enviroment 
    delta_t    = 0.01;
    init_state = [pi; pi/2; 0; 0];
    reward     = @(x,u)(norm(x)); % dummy reward
    env        = Env.TwoRRobot(init_state,delta_t,reward,'ConfigFile','robot_payload');

    env.Render();

    for ii=1:size(state,1)

        env.UpdateRender(state(ii,:)');
        pause(30/1000)

    end
end

if(strcmp(control_mode,"tracker"))
    ft         = 50; 
    delta_t    = 0.01;  
    t          = 0:delta_t:ft;
    % reference (TODO reference class)-----------------------------------------
    % sin traj
    q1des_t  = pi/2*sin(t);
    q2des_t  = pi/3*cos(t);
    dq1des_t = pi/2*cos(t);
    dq2des_t = -pi/3*sin(t);
    
    figure
    plot(q1des_t')
    hold on
    plot(state_cpp(:,1));
    
    figure
    plot(q2des_t')
    hold on
    plot(state_cpp(:,2));
    
end







