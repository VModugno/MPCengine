classdef TwoRRobot < Env.AbstractEnv
    
    
    methods
        function obj = TwoRRobot(init_state,dt,reward,varargin)
            obj.num_state            = 4;
            obj.state_bounds(1,:)    = [-2*pi,2*pi];
            obj.state_bounds(2,:)    = [-2*pi,2*pi];
            obj.state_bounds(3,:)    = [-100,100];
            obj.state_bounds(4,:)    = [-100,100];
            obj.init_state           = init_state;
            obj.state                = init_state;
            obj.state_name           = ["theta1" "theta2" "theta1_dot" "theta2_dot"];
            obj.dt                   = dt;
            obj.reward               = reward;
            obj.active_visualization = false;
            obj.Load_parameters();
            if(~isempty(varargin))
                if(strcmp(varargin{1},'ConfigFile'))
                    obj.prm = Utils.CopyPrmFromFile(varargin{2},obj.prm);
                end
            end
        end
        
        function [new_state, mes_acc] = Dynamics(obj,state,action)
            % Robot with motor at the joint dynamics. IN - [angle,rate] & torque.
            % OUT - [rate,accel]
            
            if action(1) > obj.prm.tq_saturation(1)
                action(1) = obj.prm.tq_saturation(1);
            end
            if action(2) > obj.prm.tq_saturation(2)
                action(2) = obj.prm.tq_saturation(2);
            end
            if action(1) < -obj.prm.tq_saturation(1)
                action(1) = -obj.prm.tq_saturation(1);
            end
            if action(2) < -obj.prm.tq_saturation(2)
                action(2) = -obj.prm.tq_saturation(2);
            end
            
            
            
            M = obj.get_dyn_M_2R_massmatrix(state,obj.prm);
            C = obj.get_dyn_C_2R_Coriolisvector(state,obj.prm);
            g = obj.get_dyn_g_2R_gravityvector(state,obj.prm);
    
            
            mes_acc = M\(action - C - g);
            
            
            new_state = [state(3); ...
                        state(4);...
                        mes_acc(1);...
                        mes_acc(2)];
        end
        
        function Render(obj)
            %% Set up the pendulum plot
            obj.visualization.panel = figure;
            obj.visualization.panel.Position = [680 558 400 400];
            obj.visualization.panel.Color = [1 1 1];
            
            hold on
    
            obj.visualization.size = 0.2;
            obj.visualization.line1 = line([0  sin(obj.init_state(1))], [0  cos(obj.init_state(1))], 'lineWidth', 2); % link 1
            obj.visualization.line2 = line([sin(obj.init_state(1))  sin(obj.init_state(1))+sin(obj.init_state(2))], [cos(obj.init_state(1)) cos(obj.init_state(1))+cos(obj.init_state(2))], 'lineWidth', 2); % link 2
%             obj.visualization.line3 = line([sin(obj.init_state(1))+sin(obj.init_state(2)) sin(obj.init_state(1))+sin(obj.init_state(2)) + 0.2*sin(obj.init_state(2) - pi+pi/2)], [cos(obj.init_state(1))+cos(obj.init_state(2)) cos(obj.init_state(1))+cos(obj.init_state(2)) + 0.2*cos(obj.init_state(2) - pi + pi/2)], 'lineWidth', 2); % e-e 1
%             obj.visualization.line3.Color = 'green';
%             obj.visualization.line4 = line([sin(obj.init_state(1))+sin(obj.init_state(2)) sin(obj.init_state(1))+sin(obj.init_state(2)) + 0.2*sin(obj.init_state(2) + pi/4)], [cos(obj.init_state(1))+cos(obj.init_state(2)) cos(obj.init_state(1))+cos(obj.init_state(2)) + 0.2*cos(obj.init_state(2) + pi/4)], 'lineWidth', 2); % e-e 2
%             obj.visualization.line4.Color = 'red';
            axis equal
            axis([-4 4 -3 3])
            obj.visualization.f = plot(0,0,'b','LineWidth',10); 
            

            hold off
            
            obj.active_visualization = true;
        end
        
        function UpdateRender(obj,state)
            set(obj.visualization.line1,'XData',[0  sin(state(1))]);
            set(obj.visualization.line1,'YData',[0  cos(state(1))]);
            set(obj.visualization.line2,'XData',[sin(state(1))  sin(state(1))+sin(state(2))]);
            set(obj.visualization.line2,'YData',[cos(state(1))  cos(state(1))+cos(state(2))]);
%             set(obj.visualization.line3,'XData',[sin(state(1))+sin(state(2)) sin(state(1))+sin(state(2))+0.2*sin(state(2) - pi+pi/2)]);
%             set(obj.visualization.line3,'YData',[cos(state(1))+cos(state(2)) cos(state(1))+cos(state(2))+0.2*cos(state(2) - pi+pi/2)]);
%             set(obj.visualization.line4,'XData',[sin(state(1))+sin(state(2)) sin(state(1))+sin(state(2))+0.2*sin(state(2) + pi/4)]);
%             set(obj.visualization.line4,'YData',[cos(state(1))+cos(state(2)) cos(state(1))+cos(state(2))+0.2*cos(state(2) + pi/4)]);
            drawnow;
         end
        
        function state = Wrapping(obj,state)
            % no wrapping for now
        end
        
        function M = get_dyn_M_2R_massmatrix(obj,state,prm)
            M = zeros(2,2);
            M(1,1) = prm.J1zz + prm.J2zz + prm.l1^2*prm.m1 + prm.l1^2*prm.m2 + prm.l2^2*prm.m2 + 2*prm.c1x*prm.l1*prm.m1 + 2*prm.c2x*prm.l2*prm.m2 + 2*prm.c2x*prm.l1*prm.m2*cos(state(2)) + 2*prm.l1*prm.l2*prm.m2*cos(state(2)) - 2*prm.c2y*prm.l1*prm.m2*sin(state(2));
            M(1,2) = prm.J2zz + prm.l2^2*prm.m2 + 2*prm.c2x*prm.l2*prm.m2 + prm.c2x*prm.l1*prm.m2*cos(state(2)) + prm.l1*prm.l2*prm.m2*cos(state(2)) - prm.c2y*prm.l1*prm.m2*sin(state(2));
            M(2,1) = M(1,2);
            M(2,2) = prm.m2*prm.l2^2 + 2*prm.c2x*prm.m2*prm.l2 + prm.J2zz;
        end
        
        function S = get_dyn_S_2R_Coriolismatrix(obj,state,prm)
            S = zeros(2,2);state

            S(1,1) = -state(4)*prm.l1*prm.m2*(prm.c2y*cos(state(2)) + prm.c2x*sin(state(2)) + prm.l2*sin(state(2)));
            S(1,2) = -prm.l1*prm.m2*(state(3) + state(4))*(prm.c2y*cos(state(2)) + prm.c2x*sin(state(2)) + prm.l2*sin(state(2)));
            S(2,1) = state(3)*prm.l1*prm.m2*(prm.c2y*cos(state(2)) + prm.c2x*sin(state(2)) + prm.l2*sin(state(2)));
            S(2,2) = 0;
        end 
        
        function C = get_dyn_C_2R_Coriolisvector(obj,state,prm)
            C = zeros(2,1);
            C(1,1) = -state(4)*prm.l1*prm.m2*(2*state(3) + state(4))*(prm.c2y*cos(state(2)) + prm.c2x*sin(state(2)) + prm.l2*sin(state(2)));
            C(2,1) = state(3)^2*prm.l1*prm.m2*(prm.c2y*cos(state(2)) + prm.c2x*sin(state(2)) + prm.l2*sin(state(2)));
        end
        
        function g = get_dyn_g_2R_gravityvector(obj,state,prm)
            g = zeros(2,1);
            g0 = 9.80665;
            g(1,1) = g0*prm.m1*(prm.c1x*cos(state(1)) + prm.l1*cos(state(1)) - prm.c1y*sin(state(1))) + g0*prm.m2*(prm.c2x*cos(state(1) + state(2)) + prm.l2*cos(state(1) + state(2)) - prm.c2y*sin(state(1) + state(2)) + prm.l1*cos(state(1)));
            g(2,1) = g0*prm.m2*(prm.c2x*cos(state(1) + state(2)) + prm.l2*cos(state(1) + state(2)) - prm.c2y*sin(state(1) + state(2)));
        end
        
        %% this function is only used by the class Estimated Model and the feedback linearization controller
        function dynComp = GetDynamicalComponents(obj,state)
            dynComp.S  = obj.get_dyn_S_2R_Coriolismatrix(state,obj.prm);
            dynComp.M  = obj.get_dyn_M_2R_massmatrix(state,obj.prm);
            dynComp.C  = obj.get_dyn_C_2R_Coriolisvector(state,obj.prm);
            dynComp.g  = obj.get_dyn_g_2R_gravityvector(state,obj.prm);
        end
      
         function acc = GetMeasuredAcc(obj)   
                acc = obj.measured_acc;
         end
         
         function tau = GetMeasuredAction(obj)
              M   = obj.get_dyn_M_2R_massmatrix(obj.state,obj.prm);
              C   = obj.get_dyn_C_2R_Coriolisvector(obj.state,obj.prm);
              g   = obj.get_dyn_g_2R_gravityvector(obj.state,obj.prm);
              tau = M*obj.measured_acc + C + g;
         end
        
        function Load_parameters(obj)
            %%  "actual" dynamic parameters
            obj.prm.l1 = 1;
            obj.prm.l2 = 0.5;

            obj.prm.m1 = 3;
            obj.prm.m2 = 2;
            obj.prm.c1x = -0.6;
            obj.prm.c1y = 0.01;
            obj.prm.c1z = 0;
            obj.prm.c2x = -0.2;
            obj.prm.c2y = 0.02;
            obj.prm.c2z = 0;
            obj.prm.J1zz = 1/12*obj.prm.m1*obj.prm.l1^2 + obj.prm.m1*obj.prm.c1x^2 + obj.prm.m1*obj.prm.c1y^2;
            obj.prm.J2zz = 1/12*obj.prm.m2*obj.prm.l2^2 + obj.prm.m2*obj.prm.c2x^2 + obj.prm.m2*obj.prm.c2y^2;
            obj.prm.tq_saturation = [1e10;1e10];
        end
        
        
        function LocalGenEnvParametersFile(obj,pNode,entry_node)
            
           name_node = pNode.createElement('l1');
           name_text = pNode.createTextNode(num2str(obj.prm.l1));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('l2');
           name_text = pNode.createTextNode(num2str(obj.prm.l2));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('m1');
           name_text = pNode.createTextNode(num2str(obj.prm.m1));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('m2');
           name_text = pNode.createTextNode(num2str(obj.prm.m2));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('c1x');
           name_text = pNode.createTextNode(num2str(obj.prm.c1x));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('c1y');
           name_text = pNode.createTextNode(num2str(obj.prm.c1y));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('c1z');
           name_text = pNode.createTextNode(num2str(obj.prm.c1z));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('c2x');
           name_text = pNode.createTextNode(num2str(obj.prm.c2x));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('c2y');
           name_text = pNode.createTextNode(num2str(obj.prm.c2y));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('c2z');
           name_text = pNode.createTextNode(num2str(obj.prm.c2z));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('J1zz');
           name_text = pNode.createTextNode(num2str(obj.prm.J1zz));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('J2zz');
           name_text = pNode.createTextNode(num2str(obj.prm.J2zz));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
        end
        
        
    end
end