classdef (Abstract) AbstractEnv < matlab.mixin.Copyable
   
    properties (Abstract)
        num_state
        state_bounds
        init_state
        state                    % current state (always tall vector)
        state_name
        dt
        reward
        visualization
        active_visualization
        prm                      % struct of parameter of the current model  
        measured_acc             % updated inside the step function
    end
    
    methods
        function [new_state, reward, done] = Step(obj,action)
            % to fix
            substeps = 2;
            %runge kutta 4
            for i = 1:substeps
               [k1, obj.measured_acc] = obj.Dynamics(obj.state,action);
                k2                    = obj.Dynamics(obj.state+obj.dt/2*k1,action);
                k3                    = obj.Dynamics(obj.state+obj.dt/2*k2,action);
                k4                    = obj.Dynamics(obj.state+obj.dt*k3,action);

                new_state = obj.state + obj.dt/6*(k1 + 2*k2 + 2*k3 + k4);
                
                % All states wrapped to 2pi
                new_state = obj.Wrapping(new_state);
            end
            
            obj.state = new_state; % Old state = new state
            new_state = obj.state;
            reward    = obj.reward(new_state,action);
            done      = 1;
            
            if(obj.active_visualization)
                obj.UpdateRender(new_state);
            end
            
        end
        
        function Reset(obj)
            obj.state = obj.init_state;
        end
        
    end
    
    
    
    methods(Abstract)
        
        [new_state, mes_acc] = Dynamics(obj,state,action)
        Render(obj)
        UpdateRender(obj,state)
        state = Wrapping(obj,state)
        Load_parameters(obj,varargin)
        
    end    
    
        
    
    
end
