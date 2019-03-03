classdef (Abstract) AbstractEnv < matlab.mixin.Copyable
   
    properties %(Abstract)
        num_state
        state_bounds
        init_state
        state                    % current state (always tall vector)
        state_name
        dt
        trigger_update           % for some environment we need to check if it is time to update the model (it is necessary especially when the mpc is slower than the controller)
        reward
        visualization
        active_visualization
        prm                      % struct of parameter of the current model  
        measured_acc             % updated inside the step function
        use_euler    = false     % when the system dynamics is already discretized we can directly go for euler method
        all_states               % here i can store all the states when it is necessary (not mandatory)
        %feedback_lin = false     % with this flag for each environment i require a feedback linearization or not
    end
    
    methods
        function [new_state, reward, done] = Step(obj,action)
            
            % this fucntion do stuff only if the type is statemachine
            action = obj.ReshapeAction(action);
            
            if(~obj.use_euler)
                % to fix
                substeps = 1;
                %runge kutta 4
                for i = 1:substeps
                   [k1, obj.measured_acc] = obj.Dynamics(obj.state,action);
                    k2                    = obj.Dynamics(obj.state+obj.dt/2*k1,action);
                    k3                    = obj.Dynamics(obj.state+obj.dt/2*k2,action);
                    k4                    = obj.Dynamics(obj.state+obj.dt*k3,action);

                    new_state = obj.state + obj.dt/6*(k1 + 2*k2 + 2*k3 + k4);

                    % All states wrapped to 2pi
                    
                end
            else
                 new_state = obj.Dynamics(obj.state,action);
            end
            new_state = obj.Wrapping(new_state);
            obj.state = new_state; % Old state = new state
            new_state = obj.state;
            reward    = obj.reward(new_state,action);
            done      = 1;
            
            if(obj.active_visualization)
                obj.UpdateRender(new_state);
            end
            
        end
        
        function GenEnvParametersFile(obj,basepath,ft)
           
           filepath   = strcat(basepath,'/env_parameters.xml');
           pNode      = com.mathworks.xml.XMLUtils.createDocument('parameters');
           
           entry_node = pNode.createElement('Entry');
           pNode.getDocumentElement.appendChild(entry_node);
          
           name_node = pNode.createElement('delta');
           name_text = pNode.createTextNode(num2str(obj.dt));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('ft');
           name_text = pNode.createTextNode(num2str(ft));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           % particular care has to be taken for the array in order to be
           % properly read in c++
           string = mat2str(obj.init_state);
           string = erase(string,["[","]"]);
           string = strrep(string,";"," ");
           name_node = pNode.createElement('init_state');
           name_text = pNode.createTextNode(string);
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           % side effect on pNode
           obj.LocalGenEnvParametersFile(pNode,entry_node);

           xmlwrite(char(filepath),pNode);   
        end
        
        function Reset(obj)
            obj.state = obj.init_state;
        end
        
        function SetState(obj,state)
            obj.state  = state;
            % clean_all_states
            obj.all_states = []; 
            % initialize with the new init state
            obj.all_states = state;
        end
        
        function ReadTriggerUp(obj,cur_trigger_up)
            obj.trigger_update = cur_trigger_up;
        end
        
    end
    
    
    
    methods(Abstract)
        action               = ReshapeAction(obj,action)
        [new_state, mes_acc] = Dynamics(obj,state,action)
        Render(obj)
        UpdateRender(obj,state)
        state = Wrapping(obj,state)
        Load_parameters(obj,varargin)
        LocalGenEnvParametersFile(obj,pNode,entry_node)
        
    end    
    
        
    
    
end
