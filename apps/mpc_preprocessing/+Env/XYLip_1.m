classdef XYLip_1 < Env.AbstractEnv
    
    properties
        A % dynamics matrix
        B % input dyamic matrix 
        C % measure matrix
        
    end
    
    methods
        function obj = XYLip_1(init_state,dt,reward,varargin)
            obj.num_state            = 14;
            obj.state_bounds(1,:)    = [-100,100];
            obj.state_bounds(2,:)    = [-100,100];
            obj.state_bounds(3,:)    = [-100,100];
            obj.state_bounds(4,:)    = [-100,100];
            obj.state_bounds(5,:)    = [-100,100];
            obj.state_bounds(6,:)    = [-100,100];
            obj.state_bounds(7,:)    = [-100,100];
            obj.state_bounds(8,:)    = [-100,100];
            obj.state_bounds(9,:)    = [-100,100];
            obj.state_bounds(10,:)   = [-100,100];
            obj.state_bounds(11,:)   = [-100,100];
            obj.state_bounds(12,:)   = [-100,100];
            obj.state_bounds(13,:)   = [-100,100];
            obj.state_bounds(14,:)   = [-100,100];
            obj.init_state           = init_state;
            obj.state                = init_state;
            obj.state_name           = ["x_com" "x_com_dot" "x_zmp" "x_left_f_pos" "x_left_f_pos_dot" "x_right_f_pos" "x_right_f_pos_dot"...
                                        "y_com" "y_com_dot" "y_zmp" "y_left_f_pos" "y_left_f_pos_dot" "y_right_f_pos" "y_right_f_pos_dot"];
            obj.dt                   = dt;
            obj.reward               = reward;
            obj.active_visualization = false;
            obj.Load_parameters()
            if(~isempty(varargin))
                if(strcmp(varargin{1},'ConfigFile'))
                    obj.prm = Utils.CopyPrmFromFile(varargin{2},obj.prm);
                end
            end
            %% model setting
            omega = sqrt(9.8/obj.prm.h);
            ch    = cosh(omega*obj.dt);
            sh    = sinh(omega*obj.dt);
            A_lip = [ch, sh/omega, 1-ch; omega*sh, ch, -omega*sh; 0, 0, 1];
            B_lip = [obj.dt-sh/omega; 1-ch; obj.dt];

            % Foot model
            A_foot = eye(2) + obj.dt*[0, 1; 0, 0];
            B_foot = obj.dt*[0; 1];

            % Full system
            A_x = blkdiag(A_lip, A_foot, A_foot);
            A_y = blkdiag(A_lip, A_foot, A_foot);
            B = blkdiag(B_lip, B_foot, B_foot);
            obj.A = blkdiag(A_x, A_y);
            obj.B = blkdiag(B, B);
            % i need to set it to true because in this way i will go
            % directly with euler and the discretized system
            obj.use_euler = true;
            
            % for visualization purpose
            obj.all_states = [];
            
        end
        
        function [new_state, mes_acc] = Dynamics(obj,state,action)
            % TODO verify this thing with the dynamics (for now we set mes
            % acc to zero)
            mes_acc   = [0];
            new_state = obj.A*state + obj.B*action;
        end
        
        function Render(obj)
            %% Set up the pendulum plot
            obj.visualization.panel = figure;
            obj.visualization.panel.Position = [680 558 400 400];
            obj.visualization.panel.Color = [1 1 1];
            
            
            obj.all_states = [obj.all_states,obj.init_state];
            
            hold on;
            plot(obj.init_state([1,3,4,6],:)', obj.init_state(obj.num_state/2+[1,3,4,6],:)')
            obj.visualization.footRect = [-obj.prm.footSize_x,  obj.prm.footSize_x, obj.prm.footSize_x, -obj.prm.footSize_x;
                                          -obj.prm.footSize_x, -obj.prm.footSize_x, obj.prm.footSize_x,  obj.prm.footSize_x];
            p1 = patch(obj.init_state(4,end)+obj.visualization.footRect(1,:), obj.init_state(obj.num_state/2+4,end)+obj.visualization.footRect(2,:), 'r');
            p2 = patch(obj.init_state(6,end)+obj.visualization.footRect(1,:), obj.init_state(obj.num_state/2+6,end)+obj.visualization.footRect(2,:), 'r');
            set(p1,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
            set(p2,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
            axis equal; axis([-0.5 0.5 -0.5 0.5]);
            legend('pos', 'zmp', 'footL', 'footR')
            drawnow
            

            hold off
            
            obj.active_visualization = true;
        end
        
        function UpdateRender(obj,state)
            
            obj.all_states = [obj.all_states,state];
            
            plot(obj.all_states([1,3,4,6],:)', obj.all_states(obj.num_state/2+[1,3,4,6],:)')
            p1 = patch(state(4,end)+obj.visualization.footRect(1,:), state(obj.num_state/2+4,end)+obj.visualization.footRect(2,:), 'r');
            p2 = patch(state(6,end)+obj.visualization.footRect(1,:), state(obj.num_state/2+6,end)+obj.visualization.footRect(2,:), 'r');
            set(p1,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
            set(p2,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
            axis equal; axis([-0.5 0.5 -0.5 0.5]);
            legend('pos', 'zmp', 'footL', 'footR')
            drawnow
        end
        
        function state = Wrapping(obj,state)
            
        end
        
        function Load_parameters(obj)
            %%  "actual" dynamic parameters
            obj.prm.h              = 0.8;
            obj.prm.footSize_x     = 0.05;
            obj.prm.footSize_y     = 0.03;
            % dummy states (references)
            obj.prm.foot_to_foot_x = 0;
            obj.prm.foot_to_foot_y = -0.2;
            obj.prm.vref_x         = 0.1;
            obj.prm.vref_y         = 0;
            
        end
        
        
        function LocalGenEnvParametersFile(obj,pNode,entry_node)
            
           name_node = pNode.createElement('h');
           name_text = pNode.createTextNode(num2str(obj.prm.h));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('footSize_x');
           name_text = pNode.createTextNode(num2str(obj.prm.footSize_x));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('footSize_y');
           name_text = pNode.createTextNode(num2str(obj.prm.footSize_y));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('foot_to_foot_x');
           name_text = pNode.createTextNode(num2str(obj.prm.foot_to_foot_x));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('foot_to_foot_y');
           name_text = pNode.createTextNode(num2str(obj.prm.foot_to_foot_y));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('vref_x');
           name_text = pNode.createTextNode(num2str( obj.prm.vref_x));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('vref_y');
           name_text = pNode.createTextNode(num2str( obj.prm.vref_y));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
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