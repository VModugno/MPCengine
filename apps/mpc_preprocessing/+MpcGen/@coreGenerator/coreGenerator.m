%% design choice: i would like to split variables in two different class:
%%                inner_variables = (in this order) current_state, last_control, current_ref, (we need them bot for fixed and ltv mpc)
%%                outer_variables = model parameters that we want to optimize with an external optimization loop (such as cmaes, or other black box optimization)

%% idee:
%% ristrutturare classi
%% partire dal presupposto che le matrici sono cell array (partire dal caso piu esteso cioe statemachine)
%% rendere classi mutable constraints e statemachine(maybe)
%% fare un uso razionale delle funzioni abstract per forzare la costruzione di classi applicative
%% individuare porzioni di codice che possono essere riutilizzate massimizzandole ma tenendo il codice flessibile
%% 

classdef coreGenerator <  handle
    
    properties 
        % general information about path and type of solver
        basepath                 % to identify the folder to write the generated functions
        type                     % fixed, LTV or statemachine
        solver                   % generate functions for target solver (qpoases)
        m_c                      % mutable constraints aka m_c is the structure that contains all the data about the mutable constraints
        m_c_flag                 % this is a flag tha represents if the the constraints change over time or not (case for gait generation)
        state_machine            % structure to manage state machine problem
        problemClass             % tracker or regulator
        % structure of the problem
        n                       % state space dim
        m                       % control space dim 
        q                       % output space dim 
        q_constr                % output space dim (new version of output space) for objective function
        q_obj                   % output space dim (new version of output space) for constraints
        nVariables_batch        % total numer of variables in the prediction windows
        nConstraints_batch      % totale number of constraints in the prediction windows
        delta                   % sampling time of mpc (in general different from the sampling time of the environment) in the model is called internal dt
        control_delta           % sampling time of the simulation (or for real appliction of the control loop)
        N                       % widht of prediction window
        N_constr                % number of constraints
        iteration_counter       % this counter is used for managing the case when the prediction frequency is lower than the control frequency (it increase everythime)
        actual_iteration_counter% this counter increase every time i move over the sample of the prediction windows (if internal frequency and control frequency are the same it coincides with the former counter)
        current_pred_win        % this iterator tell me in which iteration window the mpc is still operating                     
        trigger_update          % this flag it's true when the control time corresponds to the internal time of the mpc (it is usefull when the controller is faster that the mpc)                      
                                
        
        B_In        % input  bound (B_In.max and B_In.min) if B_In.min empty the framework will automatically assume   -B_In.max <u< B_In.max    
        B_Out       % output bound(B_Out.max and B_Out.min) if B_Out.min empty the framework will automatically assume   -B_Out.max <u< B_Out.max
        
        %for code generation (symbolic variables)
        sym_H                  %             
        sym_F_tra              %
        sym_G                  %
        sym_W                  %
        sym_S                  %
        index                  % inner variables for matrices that change every sample inside a prediction window (in order to select the different structure) it corresponds to actual_iteration_counter
        index_pred_window      % inner variables that let us define select different matrices for different prediction windows
        x_0                    % inner_variables
        u_prev                 % inner_variables (it is used only for tracking and represents the previous control)
        ref_0                  % inner_variables
        inner_x_ext            % variable used to get the trajectories from the oracle (for LTV propagation) 
        outer_x                % parameters to optimize that do not belong to the mpc variables
        extern_var             % "true" or "false"
        extern_dim             % dimension of external variable vector to optimize
        non_standard_iteration       % array of number that tell the code generator which is a non standard generation
        non_standard_iteration_flag  % with this variable im going to signal the qpoases fucntion generator that im going trough a special non standard value in the iteration
        % for control
        H
        F_tra
        G
        W
        S
        cur_W                  % auxiliary variable introduce to omogenize the input to the controller
        cur_G
        cur_S
        
        % link to function pointer
        propModelCall
        costFuncCall
        constrFuncG_Call
        constrFuncS_Call
        constrFuncW_Call
        
        
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
       
        function obj = coreGenerator(type,solver,generate_functions,A_cont,B_cont,C_cont_obj,C_cont_constr,N,delta,ctrl_delta,...
                                     state_machine,non_standard_iteration,function_list)
            if(~strcmp(type,'statemachine'))
                obj.n             = size(A_cont,1);
                obj.m             = size(B_cont,2);
                obj.q             = size(C_cont,1); 
                obj.N             = N;
                obj.delta         = delta;
                obj.control_delta = ctrl_delta;
                obj.index            = sym('ind',[1,1],'real');
                obj.index_pred_window= sym('ind_pred_win',[1,1],'real');
                obj.x_0              = sym('x_0',[n,1],'real');
                obj.u_prev           = sym('u_prev',[m,1],'real');
                obj.ref_0            = sym('ref_0',[N*q,1],'real');  
                obj.type             = type;
                obj.solver           = solver;
                obj.iteration_counter        = 1;
                obj.actual_iteration_counter = 1;
                obj.current_pred_win         = 1;
                obj.trigger_update           = false;
                obj.non_standard_iteration      = non_standard_iteration;
                obj.non_standard_iteration_flag = false;
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
                
            else
                % if we are dealing with state machine
                for i= 1:state_machine.n_of_models
                    obj.n(i)       = size(A_cont{i},1);
                    obj.m(i)       = size(B_cont{i},2);
                    obj.q_constr(i) = size(C_cont_constr{i},1);
                    obj.q_obj(i)   = size(C_cont_obj{i},1);
                end
                obj.N                = N;
                obj.delta            = delta;
                obj.control_delta    = ctrl_delta;
                obj.iteration_counter        = 1;
                obj.actual_iteration_counter = 1;
                obj.current_pred_win         = 1;
                 obj.trigger_update          = false;
                obj.index                    = sym('ind',[1,1],'real');
                obj.index_pred_window        = sym('ind_pred_win',[1,1],'real');
                obj.x_0                      = sym('x_0',[obj.n(1),1],'real');
                %obj.u_prev           = sym('u_prev',[m,1],'real');
                %obj.ref_0            = sym('ref_0',[N*q,1],'real');
                obj.type                     = type;
                obj.solver                   = solver;
                obj.non_standard_iteration      = non_standard_iteration;
                obj.non_standard_iteration_flag = false;
                obj.propagationModel         = function_list.propagationModel;    % (str) name of the function that we will use 
                obj.costFunc                 = function_list.costFunc;            % (str) name of the function that we will use 
                obj.constrW                  = function_list.constrW;             % (str) name of the function that we will use 
                obj.constrG                  = function_list.constrG;             % (str) name of the function that we will use 
                obj.constrS                  = function_list.constrS;             % (str) name of the function that we will use 
                obj.GetBasePath();
                if(generate_functions)
                    if ~exist(obj.basepath,'dir')
                        mkdir(convertStringsToChars(obj.basepath));
                    end
                end
                
            end
            
            
        end
       
       
       function GetBasePath(obj)
          
           % get time stamp
           dt     = datestr(now,'mmmm dd, yyyy HH:MM:SS.FFF AM');
           % folder name
           folder = strcat(obj.solver,'_',obj.type,'_',dt);
           folder = strrep(folder, ' ', '_');
           % find location of the current folder
           pth = which('mainFunctionGen.m');
           % take only the path till the container folder of current file
           pth = fileparts(pth);
           obj.basepath = strcat(pth,'/generated_function/',folder);
       end
       
       
       function GenFunctions(obj) 
           % we need to reset the non standard iteration flag because it
           % could be used during the simulation and we need to reset it before
           % starting the func generation module
           obj.non_standard_iteration_flag = false;
           if(strcmp(obj.solver,"QPoases"))
               obj.QPOASESgenFunc()
           elseif(strcmp(obj.solver,"somethingElse"))
               
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
         
           if(length(obj.n)>1)
               string    = mat2str(obj.n);
               string    = erase(string,["[","]"]);
               name_text = pNode.createTextNode(strrep(string,";"," "));
           else
               name_text = pNode.createTextNode(int2str(obj.n));
           end
           name_node = pNode.createElement('n');
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           if(length(obj.m)>1)
               string    = mat2str(obj.m);
               string    = erase(string,["[","]"]);
               name_text = pNode.createTextNode(strrep(string,";"," "));
           else
               name_text = pNode.createTextNode(int2str(obj.m));
           end
           name_node = pNode.createElement('m');
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           if(length(obj.q_constr)>1)
               string    = mat2str(obj.q_constr);
               string    = erase(string,["[","]"]);
               name_text = pNode.createTextNode(strrep(string,";"," "));
           else
               name_text = pNode.createTextNode(int2str(obj.q_constr));
           end
           name_node = pNode.createElement('q_constr');
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           if(length(obj.q_obj)>1)
               string    = mat2str(obj.q_obj);
               string    = erase(string,["[","]"]);
               name_text = pNode.createTextNode(strrep(string,";"," "));
           else
               name_text = pNode.createTextNode(int2str(obj.q_obj));
           end
           name_node = pNode.createElement('q_obj');
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
           
           name_node = pNode.createElement('nVariables_batch');
           name_text = pNode.createTextNode(int2str(obj.nVariables_batch));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('nConstraints_batch');
           name_text = pNode.createTextNode(int2str(obj.nConstraints_batch));
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
           
           name_node = pNode.createElement('ext_dt');
           name_text = pNode.createTextNode(num2str(obj.control_delta));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('internal_dt');
           name_text = pNode.createTextNode(num2str(obj.delta));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           if(strcmp(obj.type,"statemachine"))
               % better to reset the state machine pattern
               obj.ResetStateMachinePattern();
               % here because i just need the information about the
               % dimension of the control input at each sample in the
               % prediction window
               state_machine_dim_pattern = [];
               for i=1:length(obj.state_machine.state_pattern)
                  state_machine_dim_pattern = [state_machine_dim_pattern,  obj.m(obj.state_machine.state_pattern(i))];
               end
               string    = mat2str(state_machine_dim_pattern);
               string    = erase(string,["[","]"]);
               name_text = pNode.createTextNode(strrep(string,";"," "));
               name_node = pNode.createElement('state_machine_control_dim_pattern');
               name_node.appendChild(name_text);
               entry_node.appendChild(name_node);
           end
           
           name_node = pNode.createElement('number_of_models');
           name_text = pNode.createTextNode(int2str(obj.state_machine.n_of_models));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
               
           
           
           xmlwrite(char(filepath),pNode);   
       end 
       
       function [all_A,all_B]=ComputeMatricesLTV(obj,A,B)
            obj.inner_x_ext = [];
            all_A           = cell(obj.N,1);
            all_B           = cell(obj.N,1);
            % i have always to use the same variables name inside mpcModel 
            x               = sym('x',[obj.n,1],'real');
            u               = sym('u',[obj.m,1],'real');
            disp("compute_matrix_A_B");
            for kk = 1:obj.N
                %disp("ciclooo")
                % here i create the symbolic variables
                %cur_x_name = "x_" + num2str(kk-1);
                cur_x_name = "x_" + num2str(kk);
                %cur_u_name = "u_" + num2str(kk-1);
                cur_u_name = "u_" + num2str(kk);
                cur_x = sym(cur_x_name,[obj.n,1],'real');
                cur_u = sym(cur_u_name,[obj.m,1],'real');
                
                % substitute the variables in A and B with cur_u and
                % cur_x
                cur_A = A;
                cur_B = B;

                cur_A = subs(cur_A,[x],[cur_x]);
                cur_A = subs(cur_A,[u],[cur_u]);

                cur_B = subs(cur_B,[x],[cur_x]);
                cur_B = subs(cur_B,[u],[cur_u]);

                % i store the resulting value inside all A and all B
                all_A{kk} = cur_A;
                all_B{kk} = cur_B;
                % i store the current variables inside inner_x_ext
                %if(kk==1)
                     % for regulator and tracker i will not insert inside the inner_x_ext
                     % the first x variables, a.k.a. x_0, because i want to
                     % use the x_0 already defined inside the coreGenerator
                     % class. 
                     % 
                     %obj.inner_x_ext = [obj.inner_x_ext;cur_u];
                %else
                     % the order which i store this variables is gonna
                     % be the orders that i have to observe when i pass
                     % the variables to the function
                     %obj.inner_x_ext = [obj.inner_x_ext;cur_x;cur_u];
                %end
                obj.inner_x_ext = [obj.inner_x_ext;cur_x;cur_u];

            end
       end
       
       function UpdateStateMachinePattern(obj)
            % circular buffer (in this way the state pattern behave as a circular pattern)
            obj.state_machine.state_pattern = [ obj.state_machine.state_pattern(2:end,:);obj.state_machine.state_pattern(1,:)];
       end
       function ResetStateMachinePattern(obj)
            % circular buffer (in this way the state pattern behave as a circular pattern)
            obj.state_machine.state_pattern = obj.state_machine.reset;
       end
       % each time i call this function i get one step update of
       % constraints.
       % I update m_c inside
       function UpdateConstrPattern(obj) 
            obj.m_c.footstep_pattern = [ obj.m_c.footstep_pattern(2:end);obj.m_c.footstep_pattern(1)+2];
            for i = 1:obj.m_c.N_state
                % circular buffer (each pattern with this update behave as a circular buffer)
                obj.m_c.const_pattern(:,i) = [ obj.m_c.const_pattern(2:end,i);obj.m_c.const_pattern(1,i)];
            end 
       end 
       function ResetStateConstrPattern(obj)
            obj.m_c.footstep_pattern = obj.m_c.reset_footstep;
            obj.m_c.const_pattern    = obj.m_c.reset_pattern;
       end
       
       % this function is used for simulating things inside matlab and
       % there exist a corresponding mechanism on the cpp code
       function UpdateAllPattern(obj)      
           % here we compute the fraction of time samples that i need to
           % wait in order to control the  
           relative_duration = round(obj.delta/obj.control_delta);
           % when iteration counter is a multiple of relative_duration we
           % perform the update of the pattern
           if(mod(obj.iteration_counter,relative_duration)==0)
               if(~isempty(obj.m_c.footstep_pattern))
                    obj.UpdateStateMachinePattern();
               end
               if(~isempty(obj.state_machine.state_pattern))
                    obj.UpdateConstrPattern();
               end
           end
           % update of the the internal iteration counter
           %obj.iteration_counter = obj.iteration_counter + 1;
       end
       
       function UpdateIterationCounters(obj)
           
           % when the trigger update is true we need immediatly to shut it 
           % down to false because it has to stay true only for one
           % iteration
           if(obj.trigger_update)
               obj.trigger_update          = false;
           end
           
           relative_duration = round(obj.delta/obj.control_delta);
           % this counter increases each step with no distinction
           obj.iteration_counter = obj.iteration_counter + 1;
           if(mod(obj.iteration_counter,relative_duration)==0)
               
               % here i need to update the update_trigger to tell the world
               % that we are moving by one the mpc
               obj.trigger_update          = true;
               
               % update of the the internal iteration counter
               % this counter increases only when i actually move one
               % sample over the prediction window (it happens less frequently when the mpc frequency is lower than the control frequency)
               obj.actual_iteration_counter = obj.actual_iteration_counter + 1;
               % here i update the internal iteration window each time i
               % have a number of sample equal to an entire iteration
               % window
               if(mod(obj.actual_iteration_counter,obj.N)==0)
                    % i restart the actual iterator
                    obj.actual_iteration_counter = 1;
                    % i advance by one the pred window
                    obj.current_pred_win = obj.current_pred_win + 1;
               end
           end
       end
       
    end
    
    %methods(Abstract)
    %    W = MutableConstraints_W(obj,u_cur);
    %end
    
    
end