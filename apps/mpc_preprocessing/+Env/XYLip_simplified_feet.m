classdef XYLip_simplified_feet < Env.AbstractEnv
    
    properties
        A % dynamics matrix
        B % input dyamic matrix 
        C % measure matrix
        all_foot_position
        all_foot_rotation
        cur_angle
    end
    
    methods
        function obj = XYLip_simplified_feet(init_state,dt,reward,prm,varargin)
            obj.num_state            = 10;
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
            obj.state_name           = ["x_com" "x_com_dot" "x_zmp" "x_foot" "vref_x" "y_com" "y_com_dot" "y_zmp" "y_foot" "vref_y"];
            obj.dt                   = dt;
            obj.trigger_update       = false;
            obj.reward               = reward;
            obj.active_visualization = false;
            obj.prm                  = prm;
            obj.all_foot_position    = [];
            obj.all_foot_rotation    = [];
            obj.cur_angle            = [];
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
            A_x   = blkdiag(A_lip,1,1);
            A_y   = blkdiag(A_lip,1,1);
            B     = blkdiag(B_lip,1);
            B     = [B; 0 0];
            obj.A = blkdiag(A_x, A_y);
            obj.B = blkdiag(B, B);
            % i need to set it to true because in this way i will go
            % directly with euler and the discretized system
            obj.use_euler = true;
            
            % for visualization purpose
            obj.all_states = [];
            
        end
        
        function action = ReshapeAction(obj,action)
            if(length(action) == 2)
                new_action(1) = action(1);
                new_action(2) = 0;
                new_action(3) = action(2);
                new_action(4) = 0;
                action = new_action';
            else
                % here i need to check that even if the input dimension is
                % 4 i need to update the model only when it is time to move
                % to land the swinging foot. it depends on the fact that we 
                % can have a controller that is faster that the mpc so the
                % actual update
                if(~obj.trigger_update)
                    new_action(1) = action(1);
                    new_action(2) = 0;
                    new_action(3) = action(3);
                    new_action(4) = 0;
                    action = new_action';    
                end       
            end
        end
        
        function [new_state, mes_acc] = Dynamics(obj,state,action)
            % TODO verify this thing with the dynamics (for now we set mes
            % acc to zero)
            mes_acc   = [0];
            new_state = obj.A*state + obj.B*action;
        end
        
        function new_input_state = UpdateDesiredInput(obj,state)
            state(5)        = obj.x_des(1);
            state(10)       = obj.x_des(2);
            new_input_state = state;
        end
        
        function Render(obj)
            %% Set up the twoxy lip plot
            obj.visualization.panel = figure;
            obj.visualization.panel.Position = [680 558 400 400];
            obj.visualization.panel.Color = [1 1 1];
            
            
            obj.all_states = [obj.all_states,obj.init_state];
            
            hold on;
            plot(obj.init_state([1,3],:)', obj.init_state(obj.num_state/2+[1,3],:)')
%             obj.visualization.footRect = [-obj.prm.footSize_x,  obj.prm.footSize_x, obj.prm.footSize_x, -obj.prm.footSize_x;
%                                           -obj.prm.footSize_y, -obj.prm.footSize_y, obj.prm.footSize_y,  obj.prm.footSize_y];
%             p1 = patch(obj.init_state(4,end)+obj.visualization.footRect(1,:), obj.init_state(obj.num_state/2+4,end)+obj.visualization.footRect(2,:), 'r');
%             p2 = patch(obj.init_state(6,end)+obj.visualization.footRect(1,:), obj.init_state(obj.num_state/2+6,end)+obj.visualization.footRect(2,:), 'r');
%             set(p1,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
%             set(p2,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
            axis equal; axis([-0.5 0.5 -0.5 0.5]);
%            legend('pos', 'zmp', 'footL', 'footR')
            drawnow
            
            movegui([300 100]);
            
            hold off
            
            obj.active_visualization = true;
        end
        
        function UpdateRender(obj,state)
            
            obj.all_states = [obj.all_states,state];
            
            % initialization
            if(isempty(obj.all_foot_position))
                obj.all_foot_position(1,:) = [state(4) state(9)];
                %obj.all_foot_rotation(:,end) = obj.cur_theta;
            end
            if(isempty(obj.all_foot_rotation) && obj.prm.active_rotation)
                obj.all_foot_rotation(1) = obj.cur_angle;
            end
            
            %update foot position and angle
            if(obj.all_foot_position(end,1) ~= state(4) || obj.all_foot_position(end,2) ~= state(9))
                obj.all_foot_position(end+1,:) = [state(4) state(9)];
                % we embedd the angle update inside this because something
                % the angle stay fixed
                if(obj.prm.active_rotation)
                    obj.all_foot_rotation(end+1,:) = obj.cur_angle;
                end
            end
         
            subplot(3,1,1)
            plot(obj.all_states([1,3],:)', obj.all_states(obj.num_state/2 + [1 3],:)')
            
            
            
            for ii = 1:size(obj.all_foot_position(:,end),1)
            
                center_foot_x = obj.all_foot_position(ii,1);
                center_foot_y = obj.all_foot_position(ii,2);
                
                if(obj.prm.active_rotation)
                    theta = obj.all_foot_rotation(ii);
                    cur_R =[cos(theta), -sin(theta);sin(theta), cos(theta)];
                    % foot dimension coordinate
                    f_d_c = cur_R*[obj.prm.footSize_x/2;obj.prm.footSize_y/2];
                else
                     f_d_c = [obj.prm.footSize_x/2;obj.prm.footSize_y/2];
                end
                
                %x = [center_foot_x-obj.prm.footSize_x/2 center_foot_x-obj.prm.footSize_x/2 center_foot_x+obj.prm.footSize_x/2 center_foot_x+obj.prm.footSize_x/2];
                %y = [center_foot_y-obj.prm.footSize_y/2 center_foot_y+obj.prm.footSize_y/2 center_foot_y+obj.prm.footSize_y/2 center_foot_y-obj.prm.footSize_y/2];
                
                x = [center_foot_x-f_d_c(1) center_foot_x-f_d_c(1) center_foot_x+f_d_c(1) center_foot_x+f_d_c(1)];
                y = [center_foot_y-f_d_c(2) center_foot_y+f_d_c(2) center_foot_y+f_d_c(2) center_foot_y-f_d_c(2)];
                
                curr_coordinate = [x;y];
                
                p1 = patch(curr_coordinate(1,:), curr_coordinate(2,:), 'r');
                set(p1,'FaceAlpha',0.1,'EdgeColor','k','LineWidth',1,'LineStyle','-');
            end
            axis equal
            
            subplot(3,1,2)
            plot(obj.all_states([1,3,4],:)');
            
            subplot(3,1,3)
            plot(obj.all_states(obj.num_state/2 + [1,3,4],:)');
            
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
           
           name_node = pNode.createElement('vref_x');
           name_text = pNode.createTextNode(num2str( obj.prm.vref_x));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('vref_y');
           name_text = pNode.createTextNode(num2str( obj.prm.vref_y));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('single_support_duration');
           name_text = pNode.createTextNode(num2str( obj.prm.single_support_duration));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('active_rotation');
           name_text = pNode.createTextNode(num2str( obj.prm.active_rotation));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('theta_0');
           name_text = pNode.createTextNode(num2str( obj.prm.theta_0));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('angular_velocity');
           name_text = pNode.createTextNode(num2str( obj.prm.angular_velocity));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('traslation_velocity');
           name_text = pNode.createTextNode(num2str( obj.prm.traslation_velocity));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('fixed_direction');
           name_text = pNode.createTextNode(num2str( obj.prm.fixed_direction));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('duration_of_rotation');
           name_text = pNode.createTextNode(num2str( obj.prm.duration_of_rotation));
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
    
    
    
    
    
    
