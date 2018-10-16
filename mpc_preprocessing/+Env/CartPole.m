classdef CartPole < Env.AbstractEnv
    
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
        measured_acc             % updated inside the step function
    end
    
    
    methods
        function obj = CartPole(init_state,dt,reward,varargin)
            obj.num_state            = 2;
            obj.state_bounds(1,:)    = [-100,100];
            obj.state_bounds(2,:)    = [-100,100];
            obj.state_bounds(3,:)    = [-pi,pi];
            obj.state_bounds(4,:)    = [-pi,pi];
            obj.init_state           = init_state;
            obj.state                = init_state;
            obj.state_name           = ["x_c" "x_c_dot" "theta" "theta_dot"];
            obj.dt                   = dt;
            obj.reward               = reward;
            obj.active_visualization = false;
            obj.Load_parameters()
            if(strcmp(varargin{1},'ConfigFile'))
                obj.prm = Utils.CopyPrmFromFile(varargin{2},obj.prm);
            end
        end
        
        function [new_state, mes_acc] = Dynamics(obj,state,action)
            % Pendulum with motor at the joint dynamics. IN - [angle,rate] & torque.
            % OUT - [rate,accel]
            %z = z';
            g = 9.8;
            mes_acc   = [(action + obj.prm.mPend*sin(state(3))*(obj.prm.L*state(4)^2-g*cos(state(3))))/(obj.prm.mCart+obj.prm.mPend*sin(state(3))^2);...
                        (-action*cos(state(3)) - obj.prm.mPend*obj.prm.L*state(4)^2*sin(state(3))*cos(state(3)) + (obj.prm.mPend+obj.prm.mCart)*g*sin(state(3)))/(obj.prm.L*(obj.prm.mCart+obj.prm.mPend*sin(state(3))^2))];
            
            new_state = [state(2); ...
                        mes_acc(1);...
                        state(4);...
                        mes_acc(2)];
        end
        
        function Render(obj)
            %% Set up the pendulum plot
            obj.visualization.panel = figure;
            obj.visualization.panel.Position = [680 558 400 400];
            obj.visualization.panel.Color = [1 1 1];
            
            hold on
    
            obj.visualization.size = 0.2;
            obj.visualization.rect = rectangle('Position',...
                                              [obj.init_state(1)-obj.visualization.size/2, -obj.visualization.size/2, obj.visualization.size, obj.visualization.size]); % cart
            obj.visualization.line = line(obj.init_state(1) + [0  sin(obj.init_state(3))], [0  cos(obj.init_state(3))], 'lineWidth', 2); % Pendulum stick
            axis equal
            axis([-4 4 -1 1])
            obj.visualization.f = plot(0,0,'b','LineWidth',10); 
            

            hold off
            
            obj.active_visualization = true;
        end
        
        function UpdateRender(obj,state)
            set(obj.visualization.rect,'Position',[state(1)-obj.visualization.size/2, -obj.visualization.size/2, obj.visualization.size, obj.visualization.size]);
            set(obj.visualization.line,'XData',[state(1) state(1)+sin(state(3))]);
            set(obj.visualization.line,'YData',[0 cos(state(3))]);
            drawnow;
        end
        
        function state = Wrapping(obj,state)
            % All states wrapped to 2pi
            if state(3)>pi
                state(3) = -pi + (state(1)-pi);
            elseif state(3)<-pi
                state(3) = pi - (-pi - state(1));
            end
        end
        
        function Load_parameters(obj)
            %%  "actual" dynamic parameters
            obj.prm.mCart = 0.1;
            obj.prm.mPend = 0.1;
            obj.prm.L = 0.5;
        end
        
        
    end
    
    %% linearize model around the unstable equilibrium point
%     mCart = 0.1;
%     mPend = 0.1;
%     L = 0.5;
%     g = 9.8;
%     xCartMax = 5;
% 
%     % Double integrator
%     A_cont = [0 1 0 0; 0 0 -mPend*g/mCart 0; 0 0 0 1; 0 0 (mCart+mPend)*g/(L*mCart) 0];
%     B_cont = [0; 1/mCart; 0; -1/(L*mCart)];
%     C_cont = [0 1 1 1];
    
    
    
    
    
    
end