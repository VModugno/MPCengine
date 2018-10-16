classdef coreGenerator <  handle
    
    properties 
        basepath    %
        type
        solver
        %  for code generation
        sym_H       %             
        sym_F_tra   %
        sym_G       %
        sym_W       %
        sym_S       %
        x_0         %
        % for control
        H
        F_tra
        G
        W
        S
        
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
       
       
       function GenFunctions(obj) 
            if(strcmp(obj.type,"fixed") && strcmp(obj.solver,"QPoases"))
               
               %% optmization problem formulation for QPoases
               % J(z) = z'*H*z + g*z
               % s.t
               %    l_b <= z <= u_b
               %    l_b <= A*z <= u_b
               
               %% hessian cost function
               H_  = obj.sym_H(:);
               obj.cCode(H_,'compute_H',{},'H');
               obj.PostProcessFunctionForqpOASES(strcat(obj.basepath,'/compute_H.c'))
               %% linear term cost function
               g_  = (obj.x_0'*obj.sym_F_tra)'; 
               obj.cCode(g_,'compute_g',{obj.x_0},'g');
               %% linear term constraints
               A_  = obj.sym_G(:);
               obj.cCode(A_,'compute_A',{},'A');
               %% constant term constraints
               ub_ = obj.sym_W + obj.sym_S*obj.x_0;
               obj.cCode(ub_,'compute_ub',{obj.x_0},'ub');
            end         
       end
       
       function PostProcessFunctionForqpOASES(filename)
               str = fileread(filename);
               xpr = '(?<=(^|\n))[ ]*201[4567].+?(?=($|[ ]*201[4567]))';
               blocks = regexp( str, xpr, 'match' );
               
       end
       
    end
    
    
end