%% design choice: i would like to split variables in two different class:
%%                inner_variables = (in this order) current_state, last_control, current_ref, (we need them bot for fixed and ltv mpc)
%%                outer_variables = model parameters that we want to optimize with an external optimization loop (such as cmaes, or other black box optimization)


classdef coreGenerator <  handle
    
    properties 
        % general information about path and type of solver
        basepath                 % to identify the folder to write the generated functions
        type                     % fixed or LTV
        solver                   % generate functions for target solver (qpoases)
        m_c                      % mutable constraints aka m_c is the structure that contains all the data about the mutable constraints
        m_c_flag                 % this is a flag tha represents if the the constraints change over time or not (case for gait generation)
        problemClass             % tracker or regulator
        % structure of the problem
        n           % state space dim
        m           % control space dim 
        q           % output space dim 
        delta       % sampling time of the controller (in general different from the sampling time of the enviroment) in the model is called internal dt
        N           % widht of prediction window
        N_constr    % number of constraints
        
        %  for code generation
        sym_H           %             
        sym_F_tra       %
        sym_G           %
        sym_W           %
        sym_S           %
        index           % inner variables for mutable constraints (in order to select the different W structure)
        x_0             % inner_variables
        u_0             % inner_variables
        ref_0           % inner_variables
        outer_x         % parameters to optimize that do not belong to the mpc variables
        extern_var      % "true" or "false"
        extern_dim      % dimension of external variable vector to optimize
        % for control
        H
        F_tra
        G
        W
        S
        
        inner_x_ext            % variable used to get the trajectories from the oracle (for LTV propagation) 
        propagationModel       % (str) name of the function that we will use 
        costFunc               % (str) name of the function that we will use 
        constrW                % (str) name of the function that we will use 
        constrG                % (str) name of the function that we will use 
        constrS                % (str) name of the function that we will use 
        MutableConstraints_W   % function handle to the mutable constraints W
        MutableConstraints_G   % function handle to the mutable constraints G
        MutableConstraints_S   % function handle to the mutable constraints S
       
        
    end
    
%     methods(Abstract) 
%        
%     end
   
    methods
       
        function obj = coreGenerator(type,solver,generate_functions,n,m,q,N,function_list)
            obj.index            = sym('ind',[1,1],'real');
            obj.x_0              = sym('x_0',[n,1],'real');
            obj.u_0              = sym('u_0',[m,1],'real');
            obj.ref_0            = sym('ref_0',[N*q,1],'real');
            obj.type             = type;
            obj.solver           = solver;
            obj.propagationModel = function_list.propagationModel;    % (str) name of the function that we will use 
            obj.costFunc         = function_list.costFunc;            % (str) name of the function that we will use 
            obj.constrW          = function_list.constrW;             % (str) name of the function that we will use 
            obj.constrG          = function_list.constrG;             % (str) name of the function that we will use 
            obj.constrS          = function_list.constrS;             % (str) name of the function that we will use 
            obj.GetBasePath();
            if(generate_functions)
                if ~exist(obj.basepath,'dir')
                    mkdir(convertStringsToChars(obj.basepath));
                end
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
           if(strcmp(obj.solver,"QPoases"))
               %% optmization problem formulation for QPoases
               %% inportant for QPoases matrix has to be stored row wise
               % J(z) = z'*H*z + g*z
               % s.t
               %    l_b <= z <= u_b
               %    l_b <= A*z <= u_b
               %% here i define the structure of the inner_x
               if(strcmp(obj.problemClass,"tracker"))
                    inner_x = [obj.x_0;obj.u_0;obj.ref_0;obj.index];
               else
                   % if the system is not LTV obj.inner_x_ext is gonna be
                   % zero
                    inner_x = [obj.x_0;obj.inner_x_ext;obj.index];
               end
               %% hessian cost function 
               disp('generating H')
               %H_  = obj.sym_H(:);
               H_  = obj.sym_H';
               H_  = H_(:);
               obj.cCode(H_,'compute_H','current_func',{inner_x,obj.outer_x},'H');
               obj.PostProcessFunctionForqpOASES('compute_H','H')
               %% linear term cost function
               disp('generating g')
               if(strcmp(obj.problemClass,"tracker"))
                    g_  = ([obj.ref_0;obj.x_0;obj.u_0]'*obj.F_tra)'; 
               else
                    g_  = (obj.x_0'*obj.sym_F_tra)';
               end
               obj.cCode(g_,'compute_g','current_func',{inner_x,obj.outer_x},'g');
               obj.PostProcessFunctionForqpOASES('compute_g','g')
               %% linear term constraints
               disp('generating A')
               if(strcmp(obj.m_c.g,"pattern"))
                   obj.MutableConstraints_QPOASES([],'compute_A','current_func',{inner_x,obj.outer_x},'A');
               else
                   A_  = obj.sym_G';
                   A_  = A_(:);
                   obj.cCode(A_,'compute_A','current_func',{inner_x,obj.outer_x},'A');
                   obj.PostProcessFunctionForqpOASES('compute_A','A')
               end
               %% constant term constraints
               disp('generating ub')
               if(strcmp(obj.problemClass,"tracker"))
                   if(strcmp(obj.m_c.w,"pattern") || strcmp(obj.m_c.s,"pattern"))
                        % with this functions i write the function and i correct it 
                        obj.MutableConstraints_QPOASES([obj.x_0;obj.u_0],'compute_ub','current_func',{inner_x,obj.outer_x},'ub');
                   else
                        ub_ = obj.sym_W + obj.sym_S*[obj.x_0;obj.u_0];
                        % with this functions i write the function and i correct it 
                        obj.cCode(ub_,'compute_ub','current_func',{inner_x,obj.outer_x},'ub');
                        obj.PostProcessFunctionForqpOASES('compute_ub','ub') 
                   end
               else
                    if(strcmp(obj.m_c.w,"pattern") || strcmp(obj.m_c.s,"pattern"))
                        % with this functions i write the function and i correct it 
                        obj.MutableConstraints_QPOASES(obj.x_0,'compute_ub','current_func',{inner_x,obj.outer_x},'ub');
                    else
                        ub_ = obj.sym_W + obj.sym_S*obj.x_0;
                        % with this functions i write the function and i correct it 
                        obj.cCode(ub_,'compute_ub','current_func',{inner_x,obj.outer_x},'ub');
                        obj.PostProcessFunctionForqpOASES('compute_ub','ub') 
                    end
               end
           end
       end
       
       function PostProcessFunctionForqpOASES(obj,namefunc,output)
           %% working on .c file
           filepath            = strcat(obj.basepath,'/',namefunc,'.c');
           str                 = fileread(char(filepath));
           delimiter_for_split = strcat("double ",output,'[]');
           % i split at the signature of the function
           new_func1           = split(str,delimiter_for_split);
           % i split again to get the dimension of the output vector
           new_func2           = split(new_func1{2},[",",")"]);
           % here i create all the pieces fo the new function that im
           % going to stitch togheter
           new_variable_name   = strcat(output,'_out');
           new_variable_signa  = strcat("double ",new_variable_name);
           variable_declare    = strcat("double ",output,'[1]',new_func2{1});
           vector_dimension    = erase(new_func2{1},["[","]"]);
           copy_to_out         = strcat("memcpy(",new_variable_name,",",output,"[0]",",sizeof(double)*",vector_dimension,");");
           % last split 
           new_func3           = split(new_func1{2},["{","}"]);

           % reconstruct new func for .cpp
           new_string = "#include ""string.h"" " + newline + new_func1{1} + new_variable_signa +new_func3{1}+ "{" + newline + variable_declare + ";" + newline + new_func3{2} ...
                        + newline + copy_to_out  + newline + "}";

           % save the new func as .cpp
           new_name_file = strcat(obj.basepath,'/',namefunc,'.cpp');
           fid = fopen(new_name_file,'w');
           fprintf(fid,'%s',new_string);
           fclose(fid);

           % memcpy(H_out,H[0], sizeof(double)*64);

           % delete old .c file
           delete(char(filepath));

           %% working on .h file
           filepath            = strcat(obj.basepath,'/',namefunc,'.h');
           str                 = fileread(char(filepath));
           delimiter_for_split = strcat("double ",output,'[]',new_func2{1});
           new_variable_signa  = strcat("double ",new_variable_name,new_func2{1});
           new_func1           = split(str,delimiter_for_split);
           % change the signature of the function in the header file
           % important!!! in this function it is expecting a function
           % name inside the comment at the beggining of the header too
           new_string = new_func1{1} + new_variable_signa + new_func1{2} + new_variable_signa + new_func1{3};
           % save the new func as .h
           fid = fopen(filepath,'w');
           fprintf(fid,'%s',new_string);
           fclose(fid);              
       end
       
       function GenParametersFile(obj)
           
           filepath   = strcat(obj.basepath,'/parameters.xml');
           
           pNode      = com.mathworks.xml.XMLUtils.createDocument('parameters');
           
           entry_node = pNode.createElement('Entry');
           pNode.getDocumentElement.appendChild(entry_node);
         
           name_node = pNode.createElement('n');
           name_text = pNode.createTextNode(int2str(obj.n));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('m');
           name_text = pNode.createTextNode(int2str(obj.m));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('q');
           name_text = pNode.createTextNode(int2str(obj.q));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           %name_node = pNode.createElement('delta');
           %name_text = pNode.createTextNode(int2str(obj.delta));
           %name_node.appendChild(name_text);
           %entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('N');
           name_text = pNode.createTextNode(int2str(obj.N));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('N_constr');
           name_text = pNode.createTextNode(int2str(obj.N_constr));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('type');
           name_text = pNode.createTextNode(obj.type);
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('external_x');
           name_text = pNode.createTextNode(obj.extern_var);
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('external_dim');
           name_text = pNode.createTextNode(int2str(obj.extern_dim));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           xmlwrite(char(filepath),pNode);   
       end 
       
       % each time i call this function i get one step update of
       % constraints.
       % I update m_c inside
       function UpdateConstrPattern(obj)
            for i = 1:obj.m_c.N_state
                % circular buffer (each pattern with this update behave as a circular buffer)
                obj.m_c.const_pattern(:,i) = [ obj.m_c.const_pattern(2:end,i);obj.m_c.const_pattern(1,i)];
            end
            
       end  
       
    end
    
    %methods(Abstract)
    %    W = MutableConstraints_W(obj,u_cur);
    %end
    
    
end