classdef Pendubot < Env.AbstractEnv
    
    methods
        function obj = Pendubot(init_state,dt,reward,varargin)
            obj.num_state            = 4;
            obj.state_bounds(1,:)    = [-pi,pi];
            obj.state_bounds(2,:)    = [-pi,pi];
            obj.state_bounds(3,:)    = [0,30];
            obj.state_bounds(4,:)    = [0,30];
            obj.init_state           = init_state;
            obj.state                = init_state;
            obj.state_name           = ['q1' 'q2' 'dq1' 'dq2'];
            obj.dt                   = dt;
            obj.reward               = reward;
            obj.active_visualization = false;
            obj.Load_parameters()
            if(strcmp(varargin{1},'ConfigFile'))
                obj.prm = Utils.CopyPrmFromFile(varargin{2},obj.prm);
            end
        end

        function action = ReshapeAction(obj,action)
            
        end
        
        function [new_state, mes_acc] = Dynamics(obj,state,action)
            % Pendulum with motor at the joint dynamics. IN - [angle,rate] & torque.
            % OUT - [rate,accel]
            
            M   = obj.get_dyn_M_pendubot(state,obj.prm);
            C   = obj.get_dyn_C_pendubot(state,obj.prm);
            phi = obj.get_dyn_phi_pendubot(state,obj.prm);
                     
            mes_acc = M\(action - C - phi);
            
            new_state = [state(3);...
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
            % for noncollocated linearization
            state(2) = wrapToPi(state(2));

        end


         
        function M = get_dyn_M_pendubot(obj,state,prm)
            M = zeros(2,2);
            M(1,1) = prm.m1*prm.lc1^2+prm.m2*(prm.l1^2+prm.lc2^2+2*prm.l1*prm.lc2*cos(state(2)))+prm.I1+prm.I2+prm.Im;
            M(1,2) = prm.m2*(prm.lc2^2+prm.l1*prm.lc2*cos(state(2)))+prm.I2;
            M(2,1) = M(1,2);
            M(2,2) = prm.m2*prm.lc2^2+prm.I2;
        end
        
        function C = get_dyn_C_pendubot(obj,state,prm)
            C = zeros(2,1);
            C(1,1) = -prm.m2*prm.l1*prm.lc2*sin(state(2))*state(4)^2 - 2*prm.m2*prm.l1*prm.lc2*sin(state(2))*state(3)*state(4);
            C(2,1) = prm.m2*prm.l1*prm.lc2*sin(state(2))*state(3)^2;
        end
        
        function phi = get_dyn_phi_pendubot(obj,state,prm)
            phi = zeros(2,1);
            g = 9.8;
            phi(1,1) = (prm.m1*prm.lc1+prm.m2*prm.l1)*g*cos(state(1)+pi/2)+prm.m2*prm.lc2*g*cos(state(1)+pi/2+state(2));
            phi(2,1) = prm.m2*prm.lc2*g*cos(state(1)+pi/2+state(2));
           
        end
        
        function dynComp = GetDynamicalComponents(obj,state)
            dynComp.M    = obj.get_dyn_M_pendubot(state,obj.prm);
            dynComp.C    = obj.get_dyn_C_pendubot(state,obj.prm);
            dynComp.phi  = obj.get_dyn_phi_pendubot(state,obj.prm);
        end
        
        function acc = GetMeasuredAcc(obj)   
                 acc = obj.measured_acc;
        end
         
        function tau = GetMeasuredAction(obj)
              M     = obj.get_dyn_M_pendubot(obj.state,obj.prm);
              C     = obj.get_dyn_C_pendubot(obj.state,obj.prm);
              phi   = obj.get_dyn_phi_pendubot(obj.state,obj.prm);
              tau   = M*obj.measured_acc + C + phi;
        end
       
        function [deltaE,KE,PE] = DeltaEnergy(obj,state,M,prm)
            q = [state(3);state(4)];
            g = 9.8;
            
            KE = 1/2*(q'*M*q);
            PE = prm.m1*g*prm.lc1*cos(state(1))+prm.m2*g*(prm.l1*cos(state(1))+prm.lc2*cos(state(1)+state(2)));
            
            Etot = KE+PE;
            Er   = prm.m1*g*prm.lc1+prm.m2*g*(prm.l1+prm.lc2);
            deltaE = Etot-Er;
        end
        
        function [deltaPE,PE] = fitness(obj,state,prm)
            q = [state(3);state(4)];
            g = 9.8;

            PE = prm.m1*g*prm.lc1*cos(state(1))+prm.m2*g*(prm.l1*cos(state(1))+prm.lc2*cos(state(1)+state(2)));
            
            Er = prm.m1*g*prm.lc1+prm.m2*g*(prm.l1+prm.lc2);
            deltaPE = Er-PE;
        end
        
        % noncollocated
        function t_bst = noncollocated_lin(obj,M,C,phi,v2)
              d1bar = M(1,2) - M(1,1)*M(2,2)/M(2,1);
              h1bar = C(1,1) - M(1,1)*C(2,1)/M(2,1);
              phi1bar = phi(1,1) - M(1,1)*phi(2,1)/M(2,1);
              
              t_bst = d1bar*v2 + h1bar + phi1bar;
        end
        
        % collocated
        function t_fbl = collocated_lin(obj,M,C,phi,v1)
              
              d2tilda = M(1,1) - M(1,2)*M(2,1)/M(2,2);
              h2tilda = C(1,1) - M(1,2)*C(2,1)/M(2,2);
              phi2tilda = phi(1,1) - M(1,2)*phi(2,1)/M(2,2);
              
              t_fbl = d2tilda*v1 + h2tilda + phi2tilda;
              
        end  
        
%       % collocated + noncolocated 
        function [tc,v2,v3,k] = switching_lin(obj,M,C,phi,action)
              v2 = action(1); v3 = action(2); k = action(3);
              
              d1bar = M(1,2) - M(1,1)*M(2,2)/M(2,1);
              d2tilda = M(1,1) - M(1,2)*M(2,1)/M(2,2);
              beta = M(1,2)*(k*(C(2,1)+phi(2,1))-(C(2,1)+phi(2,1)))/M(2,2)-M(1,1)*(k*(C(2,1)+phi(2,1)))/M(2,1)+C(1,1)+phi(1,1);
              
              tc = d1bar*v2 + d2tilda*v3 + beta; 
        end    
        
        function Load_parameters(obj)
            %%  "actual" dynamic parameters
            % Values of Massimo
            obj.prm.m1 = 0.09;
            obj.prm.m2 = 0.08;
            obj.prm.l1 = 0.1492;
            obj.prm.l2 = 0.1905;
            obj.prm.lc1 = obj.prm.l1/2;
            obj.prm.lc2 = obj.prm.l2/2;
            obj.prm.I1 = (obj.prm.m1*obj.prm.l1^2)/12;
            obj.prm.I2 = (obj.prm.m2*obj.prm.l2^2)/12;
            obj.prm.Im = 4.74e-4;
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

                      name_node = pNode.createElement('lc1');
           name_text = pNode.createTextNode(num2str(obj.prm.lc1));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('lc2');
           name_text = pNode.createTextNode(num2str(obj.prm.lc2));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('I1');
           name_text = pNode.createTextNode(num2str(obj.prm.I1));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('I2');
           name_text = pNode.createTextNode(num2str(obj.prm.I2));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('Im');
           name_text = pNode.createTextNode(num2str(obj.prm.Im));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
                     
        end
        
        
        function [A,B,C,D] = Upright_Linearization(obj,linearization)
                 [A,B,C,D] = UprightLin.Pendubot(linearization);
        end
        
    end
    
end