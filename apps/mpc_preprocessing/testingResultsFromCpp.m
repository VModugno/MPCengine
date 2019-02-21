clear variables 
close all 
clc



%% tracking or regulator
control_mode     = "regulator"; % tracker, regulator
%% activate or deactivate visualization
visualization    = true;
%% activate or deactivate state plot againist desired trajectory or state trajectories from matlab mpc
plot_flag        = false;


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
% compute error quadratic average 
state_average_error  = sum(sum((state_cpp - all_states_gt).^2,2))/length(all_states_gt)
action_average_error = sum(sum((action_cpp- all_action_gt).^2,2))/length(all_action_gt)

%% visualization -----------------------------------------------------------------------TOFIX
if(visualization)
    state = state_cpp;
    % plot using the enviroment 
    delta_t                           = 0.05;
    prm.h                             = 0.26;%0.8;
    prm.footSize_x                    = 0.05;
    prm.footSize_y                    = 0.03;
    %prm.foot_to_foot_x                = 0.0;        % desired foot to foot distance along x
    %prm.foot_to_foot_y                = -0.2;%-0.2  % desired foot to foot distance along x
    prm.vref_x                        = 0.1;        % desired com velocity
    prm.vref_y                        = 0;
    %prm.footstep_constraints_x        = 0.1;
    prm.inner_footstep_constraints_y  = 0.20;
    prm.outer_footstep_constraints_y  = 0.25; 
    %init_state = [0; 0; pi/10; 0];
    init_state = [  0;    0; 0; 0  ;  0.1;  % sagittal axis (x coordinate) com position, com velocity, zmp position, initial footstep and vrefx
                    0.05; 0;    0.1; 0.1;   0];  % coronal  axis (y coordinate) foot position and velocity, zmp position, initial footstep and vrefy
    reward     = @(x,u)(norm(x)); % dummy reward
    env        = Env.XYLip_simplified_feet(init_state,delta_t,reward,prm);
    
    env.Render();
    
    for ii=1:size(state,1)
        
        env.UpdateRender(state(ii,:)');
        pause(30/1000)
        
    end

    % plot using the enviroment 
%     delta_t    = 0.01;
%     init_state = [pi; pi/2; 0; 0];
%     reward     = @(x,u)(norm(x)); % dummy reward
%     env        = Env.TwoRRobot(init_state,delta_t,reward,'ConfigFile','robot_payload');
% 
%     env.Render();
% 
%     for ii=1:size(state,1)
% 
%         env.UpdateRender(state(ii,:)');
%         pause(30/1000)
% 
%     end
end

%% plot ---------------------------------------------------------------------------
if(plot_flag)
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
    elseif(strcmp(control_mode,"regulator"))
        
        % blu is the cpp 
        % red is the ground truth
        for i =1:size(all_states_gt,2)
            figure
            plot(all_states_gt(:,i),'r')
            hold on
            plot(state_cpp(:,i),'b');
        end
    end
end






