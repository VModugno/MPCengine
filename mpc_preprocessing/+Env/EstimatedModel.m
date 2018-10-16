classdef EstimatedModel < handle
    
    
    properties
        env
        orig_prm
        param_fields 
    end
    
    
    methods 
        function obj = EstimatedModel(env,varargin)
            obj.env          = copy(env);
            obj.orig_prm     = obj.env.prm;
            obj.param_fields = fieldnames(env.prm);
            if(strcmp(varargin{1},'ConfigFile'))
                obj.env.prm  = Utils.CopyPrmFromFile(varargin{2},obj.env.prm);
                obj.orig_prm = obj.env.prm;
            end
        end
        
        function UpdatePrm(obj,param_list,value)
            for i = 1:numel(obj.param_fields)
                for j = 1:length(param_list)
                    if(strcmp(obj.param_fields{i},param_list(j)))
                        obj.env.prm.(obj.param_fields{i}) = value(j);
                    end
                end
            end
        end
        
        function DynComp = ComputeDynamics(obj,state)
            DynComp = obj.env.GetDynamicalComponents(state);
        end
    end
    
    
    
end