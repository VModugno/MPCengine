classdef InvPend < Env.AbstractEnv
    
    properties 
        num_state
        state_bounds
        init_state
        state
        state_name
        dt
        reward
        visualization
        active_visualization
        prm
        measured_acc            % updated inside the step function
    end
    
    
    methods
        function obj = InvPend(init_state,dt,reward,prm,varargin)
            obj.num_state            = 2;
            obj.state_bounds(1,:)    = [-pi,pi];
            obj.state_bounds(2,:)    = [-pi,pi];
            obj.init_state           = init_state;
            obj.state                = init_state;
            obj.state_name           = ["theta" "theta_dot"];
            obj.dt                   = dt;
            obj.reward               = reward;
            obj.active_visualization = false;
            obj.prm                  = prm;
            %obj.Load_parameters()
            if(strcmp(varargin{1},'ConfigFile'))
                obj.prm = Utils.CopyPrmFromFile(varargin{2},obj.prm);
            end
            
        end
        
        function action = ReshapeAction(obj,action)
            
        end
        
        function [new_state, mes_acc] = Dynamics(obj,state,action)
            % Pendulum with motor at the joint dynamics. IN - [angle,rate] & torque.
            % OUT - [rate,accel]
            g = 1;
            %z = z';
            mes_acc   = g/obj.prm.L*sin(state(1))+action;
            new_state = [state(2); mes_acc];
        end
        
        function Render(obj)
            %% Set up the pendulum plot
            obj.visualization.panel = figure;
            obj.visualization.panel.Position = [680 558 400 400];
            obj.visualization.panel.Color = [1 1 1];
            
            hold on
            % Axis for the pendulum animation
            obj.visualization.f = plot(0,0,'b','LineWidth',10); % Pendulum stick
            axPend = obj.visualization.f.Parent;
            axPend.XTick = []; % No axis stuff to see
            axPend.YTick = [];
            axPend.Visible = 'off';
            %axPend.Position = [0.01 0.5 0.3 0.3];
            axPend.Clipping = 'off';
            axis equal
            axis([-1.2679 1.2679 -1 1]);
            plot(0.001,0,'.k','MarkerSize',50); % Pendulum axis point

            hold off
            
            obj.active_visualization = true;
        end
        
        function UpdateRender(obj,state)
            % Pendulum state:
            set(obj.visualization.f,'XData',[0 -sin(state(1))]);
            set(obj.visualization.f,'YData',[0 cos(state(1))]);
            drawnow;
        end
        
        function state = Wrapping(obj,state)
            % All states wrapped to 2pi
            if state(1)>pi
                state(1) = -pi + (state(1)-pi);
            elseif state(1)<-pi
                state(1) = pi - (-pi - state(1));
            end
        end
        
        function Load_parameters(obj)
            %%  "actual" dynamic parameters
            obj.prm.L = 1;
        end
        
    end
    
    %% linearized system around the unstable equilibrium point
%     A_cont = [0 1; 1 0];
%     B_cont = [0; 1];
%     C_cont = [1 0];
    
    
    
    
end