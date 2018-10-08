classdef coreGenerator <  handle
    
    properties 
        basepath    %
        type
        solver
        sym_H       %              
        sym_F_tra   %
        sym_G       %
        sym_W       %
        sym_S       %
        x_0         %
        
    end
    
%     methods(Abstract) 
%        
%     end
   
    methods
       
        function obj = coreGenerator(type,solver)
            
            obj.type   = type;
            obj.solver = solver;
            obj.GetBasePath();
            if ~exist(obj.basepath,'dir')
                mkdir(convertStringsToChars(obj.basepath));
            end
            
            
        end
       
       
       function GetBasePath(obj)
          
           % get time stamp
           dt     = datestr(now,'mmmm dd, yyyy HH:MM:SS.FFF AM');
           % folder name
           folder = strcat(obj.solver,'_',obj.type,'_',dt);
           % find location of the current folder
           pth = which('mainFunctionGen.m');
           % take only the path till the container folder of current file
           pth = fileparts(pth);
           obj.basepath = strcat(pth,'/generated_function/',folder);
       end
       
       
       function GenFunction(obj) 
           if(strcmp(obj.type,"fixed") && strcmp(obj.solver,"QPoases"))
               H_  = obj.sym_H(:);
               g_  = (obj.x_0'*obj.sym_F_tra)'; 
               % check how to pass the constraints alredy digested for
               % qpoases in the bemprad form
               % ub_ = inv(obj.sym_G)*(obj.sym_W + obj.sym_S*obj.x_0);
               fun_name = 'computeH';
               output   = 'H';
               obj.cCode(H_,fun_name,{},output);
           end
             
       end
    end
    
    
end