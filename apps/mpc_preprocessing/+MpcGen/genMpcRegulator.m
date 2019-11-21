classdef genMpcRegulator < MpcGen.coreGenerator


    properties
           
      A
      B
      C_constr
      C_obj
      Q
      R
      u_star_debug 
      
      
      
      % for simulation on matlab we can define condition for non
      %standard iteration directly inside the matrices function. for code
      %generation we need to explicitly state that we have to deal with non
      %standard generation when we construct the cpp code. fo
        
       
        
    end



    methods
        function obj = genMpcRegulator(A_cont,B_cont,C_cont_obj,C_cont_constr,B_In,B_Out,delta,ctrl_delta,N,state_gain,control_cost,...
                                       type,solver,generate_functions,discretized,mutable_constr,state_machine,non_standard_iteration,function_list)
           
            % call super class constructor
            obj = obj@MpcGen.coreGenerator(type,solver,generate_functions,A_cont,B_cont,C_cont_obj,C_cont_constr,N,delta,ctrl_delta,...
                                           state_machine,non_standard_iteration,function_list);
            
            % problem structure
            obj.type         = type; 
            obj.solver       = solver; 
            obj.problemClass = 'regulator';
            
            obj.state_machine = state_machine;
           
            
            % when we do not have external varialbes to optimize we assign a dimension of one just to allow
            % matlab to provide the right functions signature
            % i need to set a dimension of 2 here in order to force the
            % ccode function to generate a pointer in the signature
            obj.outer_x = sym('outer_x',[2,1],'real');    
            obj.extern_var = "false";
            obj.extern_dim = 0;
            
            % i set inner_x_ext empty but if it used i can measure the
            % length of this vector to trigger "the LTV" like behaviour
            obj.inner_x_ext = []; % empty vector
            
            
            %% TODO add managment of bounds ONLY with state machine
            if(isempty(state_machine))
                if(length(B_In.max)~= obj.m)
                    % i need to avoid to rise an error for dimension mismatch
                    % when i have mutable constraints
                    if(isempty(mutable_constr))
                        error('the maxInput has to be a vector with m elements wehre m is the number of input')
                    end
                else
                    obj.B_In = B_In;
                end
                if(length(B_Out.max)~= obj.q)
                    % i need to avoid to rise an error for dimension mismatch
                    % when i have mutable constraints
                    if(isempty(mutable_constr) && isempty(state_machine))
                        error('the maxOutput has to be a vector with q elements wehre q is the number of output')
                    end
                else
                    obj.B_Out = B_Out;    
                end
            end
            
            %% Discrete system (when necessary)
            if(~isempty(obj.state_machine))
                if(~discretized)
                    for i=1:obj.state_machine.n_of_models
                        obj.A{i}        = eye(obj.n(i)) + delta*A_cont{i};
                        obj.B{i}        = obj.delta*B_cont{i};
                        obj.C_constr{i} = C_cont_constr{i};
                        obj.C_obj{i}    = C_cont_obj{i};
                    end
                else
                    obj.A        = A_cont;
                    obj.B        = B_cont;
                    obj.C_constr = C_cont_constr;
                    obj.C_obj    = C_cont_obj;
                end
            else
                if(~discretized)
                    %% tODO add different C matrix (one for measure one for constraints)
                    A = eye(obj.n) + delta*A_cont;
                    B = obj.delta*B_cont;
                    C = C_cont;
                else
                    A = A_cont;
                    B = B_cont;
                    C = C_cont;
                end
                
            end

            %% Cost Function (here i can manage both scalar and vector cost)
            if(~isempty(obj.state_machine))
                for i=1:obj.state_machine.n_of_models 
                    if(length(state_gain{i}) == obj.q_obj(i))
                        obj.Q{i} = diag(state_gain{i});
                    else
                        error('diagonal state gain has the wrong size, fix it!');
                    end
        
                    if(length(control_cost{i}) == obj.m(i))
                        obj.R{i} = diag(control_cost{i});
                    else
                        error('diagonal control cost has the wrong size, fix it!');
                    end
                    
                end
            else
                %% TODO distinguish between constraints C and Objective C
                if(length(state_gain)==1)
                    Q = state_gain*eye(obj.q);
                else
                    if(length(state_gain) == obj.q)
                        Q = diag(state_gain);
                    else
                        error('diagonal state gain has the wrong size, fix it!');
                    end
                end
                if(length(state_gain)==1)
                    R = control_cost*eye(obj.m);
                else
                    if(length(control_cost) == obj.m)
                    R = diag(control_cost);
                    else
                        error('diagonal control cost has the wrong size, fix it!');
                    end
                end
            end

            %% managing mutable constraints (this structure will be used inside funcgenerator)
            obj.m_c = mutable_constr;
            if(isempty(mutable_constr))
                obj.m_c_flag = false;
                obj.m_c.g    = "nonMut";
                obj.m_c.w    = "nonMut";
                obj.m_c.s    = "nonMut";
            else
                obj.m_c_flag = true;
                if(obj.m_c.g)
                    obj.m_c.g = "pattern";
                else
                    obj.m_c.g ="nonMut";
                end
                if(obj.m_c.w)
                    obj.m_c.w = "pattern";
                else
                    obj.m_c.w ="nonMut";
                end
                if(obj.m_c.s)
                    obj.m_c.s = "pattern";
                else
                    obj.m_c.s ="nonMut";
                end
            end
            
            %% overWrite A and B if the system is LTV and i Initialize inner_x_ext
            A = obj.A;
            B = obj.B; 
            C_constr = obj.C_constr;
            C_obj = obj.C_obj;
            Q = obj.Q;
            R = obj.R;
            
            % here i trigger the ltv
            if(strcmp(obj.type,"ltv"))
                [A,B]=obj.ComputeMatricesLTV(A,B);
            end
            %% Construct matrices (it automatically detects fixed or ltv) here for rotation i added a situation where i use inner_x_ext even if is not LTV
            obj.propModelCall             = "CostFunc.propagationModel_regulator_"+ type + "_" + obj.propagationModel + "(obj,A,B,C_obj,C_constr,Q,R)";
            [S_bar_obj,S_bar_constr,T_bar_obj,T_bar_constr,Q_bar,R_bar] = eval(obj.propModelCall);

            %% Cost function matrices
            obj.costFuncCall      = "CostFunc.regulator_" + obj.costFunc + "(S_bar_obj,T_bar_obj,Q_bar,R_bar)";
            [obj.H,obj.F_tra] = eval(obj.costFuncCall);
            %% Constraints matrices
            % g function
            % through obj.m_c.g we know if g is mutable or not 
            obj.constrFuncG_Call = "Constraint.regulator_G_"+ type + "_" +obj.m_c.g + "_" + obj.constrG + "(obj,S_bar_constr)";
            obj.G                = eval(obj.constrFuncG_Call);
             % if G is mutable i need to store the current function inside
             % a function handle of the class (needded both for func gen and compute control)
             % and i need to store the variables that G is depending upon
            if(strcmp(obj.m_c.g,"pattern"))
                str2funcCall             = "Constraint.regulator_G_" + type + "_" +obj.m_c.g + "_" + obj.constrG;
                obj.MutableConstraints_G = str2func(str2funcCall);
                obj.m_c.S_bar            = S_bar_constr;
            end
            % S function
            % through obj.m_c.s we know if g is mutable or not 
            obj.constrFuncS_Call = "Constraint.regulator_S_" + type + "_" +obj.m_c.s + "_" + obj.constrS + "(obj,T_bar_constr)";
            obj.S                = eval(obj.constrFuncS_Call);
            % if S is mutable i need to store the current function inside
            % a function handle of the class (needded both for func gen and compute control)
            % and i need to store the variables that G is depending upon
            if(strcmp(obj.m_c.s,"pattern"))
                str2funcCall             = "Constraint.regulator_S_"+ type + "_" + obj.m_c.s + "_" + obj.constrS;   
                obj.MutableConstraints_S = str2func(str2funcCall);
                obj.m_c.T_bar            = T_bar;
            end
             % w function
             % through obj.m_c.w we know if g is mutable or not 
            obj.constrFuncW_Call = "Constraint.regulator_W_" + type + "_" +obj.m_c.w + "_" + obj.constrW + "(obj)";
            obj.W                = eval(obj.constrFuncW_Call);
            % if W is mutable i need to store the current function inside
            % a function handle of the class (needded both for func gen and compute control)
            % and i need to store the variables that G is depending upon
            if(strcmp(obj.m_c.w,"pattern"))
                str2funcCall             = "Constraint.regulator_W_" + type + "_" + obj.m_c.w + "_" + obj.constrW;
                obj.MutableConstraints_W = str2func(str2funcCall);
            end
            
            %% i copy all the matrix in the sym_* variables in order to pass them to the function generation method 
            obj.sym_H      = sym(obj.H);                   
            obj.sym_F_tra  = sym(obj.F_tra); 
            obj.sym_G      = sym(obj.G);      % for the function generation it works only if G is not mutable (it should works right away both fixed and ltv)
            obj.sym_W      = sym(obj.W);      % for the function generation it works only if W is not mutable (it works right away both fixed and ltv)
            obj.sym_S      = sym(obj.S);      % for the function generation it works only if W is not mutable (it works right away both fixed and ltv)
            
            
            %% here i initialize the variables necessary for qpoases and mpcproblem in cpp
            obj.nVariables_batch    = size(obj.sym_G,2);
            obj.nConstraints_batch  = size(obj.sym_G,1);
            
            %% TODO here before continuing i need to set the value of the outer parameters by calling the update function
            
            
            %% cost function post processing
            % if the matrix has been computed for LTV i need to transform
            % them into matlab function to use them inside matlab for
            % compute control 
            if(strcmp(obj.type,"ltv"))
                obj.H     = matlabFunction(obj.H,'vars', {obj.x_0,obj.inner_x_ext});
                obj.F_tra = matlabFunction(obj.F_tra,'vars', {obj.x_0,obj.inner_x_ext});
            end
                
            
            %% constraints function post processing
            % if some of the constraints matrix are mutable i need to store
            % them inside the 
            % G post-processing
            if(strcmp(obj.m_c.g,"pattern"))
                %if(strcmp(obj.type,"ltv") )
                if(length(obj.inner_x_ext)>=1)
                    if(strcmp(obj.type,"ltv") )
                        obj.m_c.S_bar_func       = matlabFunction(obj.m_c.S_bar,'vars', {obj.x_0,obj.inner_x_ext});
                    else
                        obj.m_c.S_bar_constr     = matlabFunction(obj.m_c.S_bar,'vars', {obj.inner_x_ext});
                    end
                    obj.cur_G = obj.G;
                elseif(strcmp(obj.type,"fixed"))
                    % in the case of fixed pattern i need to initialize the
                    % current matrix value with the one computed before 
                    obj.cur_G = obj.G;     
                end      
            else
                if(strcmp(obj.type,"ltv"))
                    obj.G = matlabFunction(obj.G,'vars', {obj.x_0,obj.inner_x_ext});
                end
            end
            % S post-processing
            if(strcmp(obj.m_c.s,"pattern"))
                if(strcmp(obj.type,"ltv"))
                    obj.m_c.T_bar_func       = matlabFunction(obj.m_c.T_bar,'vars', {obj.x_0,obj.inner_x_ext});
                elseif(strcmp(obj.type,"fixed"))
                    % in the case of fixed pattern i need to initialize the
                    % current matrix value with the one computed before
                    obj.cur_S = obj.S;
                end 

            else
                if(length(obj.inner_x_ext)>=1)
                    if strcmp(obj.type,"ltv")
                        obj.S = matlabFunction(obj.S,'vars', {obj.x_0,obj.inner_x_ext});
                    else
                        obj.S = matlabFunction(obj.S,'vars', {obj.inner_x_ext});
                    end
                end
            end
            % W post-processing
            if(strcmp(obj.m_c.w,"pattern"))
                % in the case of fixed pattern i need to initialize the
                % current matrix value with the one computed before
                obj.cur_W = obj.W;
            end
            
            
            %% store number of constraints
            % here i compute the number of constraints for each step it is
            % very immportant for the Cpp version of mpc
            %% TODO to fix for cpp code generator
            obj.N_constr  = size(obj.S,1)/obj.N;
            
        end
        
        % xu_oracle_traj has to contain all the trajectory from the current
        % state to the last one in the future inside the prediction window
        % (for now we keep the oracle outside the MPC controller as a separated object)
        function tau = ComputeControl(obj,x_cur,xu_oracle_trajectory)
             if(strcmp(obj.type,"ltv"))
                 H     = obj.H(xu_oracle_trajectory);
                 F_tra = obj.F_tra(xu_oracle_trajectory);
                 if(strcmp(obj.m_c.g,"pattern"))
                    S_bar     = obj.m_c.S_bar(xu_oracle_trajectory);
                 else
                    obj.cur_G = obj.G(xu_oracle_trajectory);
                 end
                 if(strcmp(obj.m_c.s,"pattern"))
                    T_bar     = obj.m_c.T_bar(xu_oracle_trajectory);
                 else
                    obj.cur_S = obj.S(xu_oracle_trajectory);
                 end
             elseif(strcmp(obj.type,"fixed"))
                 H          = obj.H;
                 F_tra      = obj.F_tra;
                 if(strcmp(obj.m_c.g,"pattern"))
                    S_bar   = obj.m_c.S_bar;
                 else
                    obj.cur_G  = obj.G;
                 end
                 if(strcmp(obj.m_c.s,"pattern"))
                    T_bar      = obj.m_c.T_bar;
                 else
                    obj.cur_S  = obj.S;
                 end
                 if(~strcmp(obj.m_c.w,"pattern"))
                    obj.cur_W  = obj.W;
                 end
             elseif(strcmp(obj.type,"statemachine"))
                 H          = obj.H;
                 F_tra      = obj.F_tra;
                 if(strcmp(obj.m_c.g,"pattern"))
                    obj.cur_G   = double(subs(obj.G,obj.inner_x_ext,xu_oracle_trajectory));
                 else
                    obj.cur_G   = obj.G;
                 end
                 if(length(obj.inner_x_ext)>=1)
                     obj.cur_S  = obj.S(xu_oracle_trajectory);
                 else
                     obj.cur_S  = obj.S;
                 end
                 
                 %obj.cur_W  = obj.W; 
             end
             tic
             u_star = quadprog(H, x_cur'*F_tra, obj.cur_G, obj.cur_W+obj.cur_S*x_cur);%,[],[],[],[],[],options);
             toc
             if(strcmp(obj.type,"statemachine"))
                index = obj.state_machine.state_pattern(1);
                tau   = u_star(1:obj.m(index));
                % DEBUG visualizing the trace of the predicted solution
                index_control = 0;
                for i = 1:length(obj.state_machine.state_pattern)
                    current_system          = obj.state_machine.state_pattern(i);
                    current_dimension       = obj.m(current_system);
                    obj.u_star_debug{i}     = u_star(index_control+(1:current_dimension));
                    index_control           = index_control + current_dimension;
                end
                % DEBUG
             else
                 tau = u_star(1:obj.m);
             end
             
             % after each iteration we need to update the mutable (when they are present)
             % constraints and the state machine constraints
             obj.UpdateIterationCounters();
             obj.UpdateAllPattern();  
             
             if(strcmp(obj.type,"statemachine"))
                 A        = obj.A;
                 B        = obj.B;
                 C_constr = obj.C_constr;
                 C_obj    = obj.C_obj;
                 Q        = obj.Q;
                 R        = obj.R;
                 %obj.UpdateStateMachinePattern();
                 [S_bar_obj,S_bar_constr,T_bar_obj,T_bar_constr,Q_bar,R_bar] = eval(obj.propModelCall);
                 [obj.H,obj.F_tra]                                           = eval(obj.costFuncCall);
                  if(~strcmp(obj.m_c.g,"pattern"))
                    obj.G                                                    = eval(obj.constrFuncG_Call);
                  end
                  if(~strcmp(obj.m_c.s,"pattern"))
                    if(length(obj.inner_x_ext)>=1)
                        % do nothing
                    else
                        obj.S                                                    = eval(obj.constrFuncS_Call);
                    end
                  end
                  if(~strcmp(obj.m_c.w,"pattern"))
                    obj.cur_W                                                = eval(obj.constrFuncW_Call);%obj.MutableConstraints_W(obj);
                  end   
             end
             if (obj.m_c_flag)
                 % i assume that the pattern are the same for every
                 %  matrix constraint
                 %  obj.UpdateConstrPattern();
                 if(strcmp(obj.m_c.g,"pattern"))
                    if(length(obj.inner_x_ext)>=1)  
                        S_bar_constr = obj.m_c.S_bar_constr(xu_oracle_trajectory);
                    end 
                    obj.cur_G        = obj.MutableConstraints_G(obj,S_bar_constr);
                 end
                 if(strcmp(obj.m_c.s,"pattern"))
                     obj.cur_S   = obj.MutableConstraints_S(obj,T_bar);
                 end
                 if(strcmp(obj.m_c.w,"pattern"))
                    obj.cur_W   = obj.MutableConstraints_W(obj);
                 end
                 
             end
        end
        
        
        
        function GenFunctions(obj) 
             GenFunctions@MpcGen.coreGenerator(obj)  
        end
       
        function GenEnvParametersFile(obj)
            [pNode]=GenEnvParametersFile@MpcGen.coreGenerator(obj);
        end
        
    end


end
