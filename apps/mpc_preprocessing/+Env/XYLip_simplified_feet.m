classdef XYLip_simplified_feet < Env.AbstractEnv
    
    properties
        A % dynamics matrix
        B % input dyamic matrix 
        C % measure matrix
        
    end
    
    methods
        function obj = XYLip_simplified_feet(init_state,dt,reward,prm,varargin)
            obj.num_state            = 8;
            obj.state_bounds(1,:)    = [-100,100];
            obj.state_bounds(2,:)    = [-100,100];
            obj.state_bounds(3,:)    = [-100,100];
            obj.state_bounds(4,:)    = [-100,100];
            obj.state_bounds(5,:)    = [-100,100];
            obj.state_bounds(6,:)    = [-100,100];
            obj.state_bounds(7,:)    = [-100,100];
            obj.state_bounds(8,:)    = [-100,100];
            obj.init_state           = init_state;
            obj.state                = init_state;
            obj.state_name           = ["x_com" "x_com_dot" "x_zmp" "x_foot" "y_com" "y_com_dot" "y_zmp" "y_foot"];
            obj.dt                   = dt;
            obj.reward               = reward;
            obj.active_visualization = false;
            obj.prm                  = prm;
            %obj.Load_parameters()
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

    

            % Full system
            A_x = blkdiag(A_lip,1);
            A_y = blkdiag(A_lip,1);
            B   = blkdiag(B_lip,1);
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
            if(length(action) == 2)
                new_action(1) = action(1);
                new_action(2) = 0;
                new_action(3) = action(2);
                new_action(4) = 0;
                
                action = new_action';
            end
            
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
            plot(obj.init_state([1,3,4],:)', obj.init_state(obj.num_state/2+[1,3,4],:)')
%             obj.visualization.footRect = [-obj.prm.footSize_x,  obj.prm.footSize_x, obj.prm.footSize_x, -obj.prm.footSize_x;
%                                           -obj.prm.footSize_y, -obj.prm.footSize_y, obj.prm.footSize_y,  obj.prm.footSize_y];
%             p1 = patch(obj.init_state(4,end)+obj.visualization.footRect(1,:), obj.init_state(obj.num_state/2+4,end)+obj.visualization.footRect(2,:), 'r');
%             p2 = patch(obj.init_state(6,end)+obj.visualization.footRect(1,:), obj.init_state(obj.num_state/2+6,end)+obj.visualization.footRect(2,:), 'r');
%             set(p1,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
%             set(p2,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
            axis equal; axis([-0.5 0.5 -0.5 0.5]);
%            legend('pos', 'zmp', 'footL', 'footR')
            drawnow
            

            hold off
            
            obj.active_visualization = true;
        end
        
        function UpdateRender(obj,state)
            
            obj.all_states = [obj.all_states,state];
            
%             plot(obj.all_states([1,3,4],:)', obj.all_states(obj.num_state/2+[1,3,4],:)')
%             axis equal; axis([-0.5 0.5 -0.5 0.5]);

            subplot(2,1,1)
            plot(obj.all_states(obj.num_state/2 + [1,3],:)');
            
            subplot(2,1,2)
            plot(obj.all_states(obj.num_state/2 + [4],:)');
            
%             p1 = patch(state(4,end)+obj.visualization.footRect(1,:), state(obj.num_state/2+4,end)+obj.visualization.footRect(2,:), 'r');
%             p2 = patch(state(6,end)+obj.visualization.footRect(1,:), state(obj.num_state/2+6,end)+obj.visualization.footRect(2,:), 'r');
%             set(p1,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
%             set(p2,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
%             legend('pos', 'zmp', 'footL', 'footR')
            drawnow
        end
        
        function state = Wrapping(obj,state)
            disp('nothing')
        end
        
         function Load_parameters(obj)
            %%  "actual" dynamic parameters
%             obj.prm.h              = 0.8;
%             obj.prm.footSize_x     = 0.05;
%             obj.prm.footSize_y     = 0.03;
%             % dummy states (references)
%             obj.prm.foot_to_foot_x = 0;
%             obj.prm.foot_to_foot_y = -0.2;
%             obj.prm.vref_x         = 0.1;
%             obj.prm.vref_y         = 0;
              disp('nothing')
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
    
    
    
    
    
    
