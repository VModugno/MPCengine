classdef genMpcRegulator < MpcGen.coreGenerator


    properties
          
        maxInput    
        maxOutput 
        inner_x_ext
        propagationModel 
        costFunc
        constrW
        constrG
        constrS
        MutableConstraints_W   % function handle to the mutable constraints W
        MutableConstraints_G   % function handle to the mutable constraints G
        MutableConstraints_S   % function handle to the mutable constraints S
        cur_W                  % auxiliary variable introduce as a unique entry both for 
        cur_G
        cur_S
        
       
        
    end



    methods
        function obj = genMpcRegulator(A_cont,B_cont,C_cont,maxInput,maxOutput,delta,N,state_gain,control_cost,...
                                       type,solver,generate_functions,discretized,mutable_constr)
            
            % call super class constructor
            obj = obj@MpcGen.coreGenerator(type,solver,generate_functions,size(A_cont,1),size(B_cont,2),size(C_cont,1),N);
            
            % problem structure
            obj.type         = type; 
            obj.solver       = solver; 
            obj.problemClass = 'regulator';
            
            % problem dimensions
            obj.n     = size(A_cont,1);
            obj.m     = size(B_cont,2);
            obj.q     = size(C_cont,1);
            obj.N     = N;
            obj.delta = delta;
            
            % when we do not have external varialbes to optimize we assign a dimension of one just to allow
            % matlab to provide the right functions signature
            % i need to set a dimension of 2 here in order to force the
            % ccode function to generate a pointer in the signature
            obj.outer_x = sym('outer_x',[2,1],'real');    
            obj.extern_var = "false";
            obj.extern_dim = 0;
            
            if(length(maxInput)~= obj.m)
                % i need to avoid to rise an error for dimension mismatch
                % when i have mutable constraints
                if(isempty(mutable_constr))
                    error('the maxInput has to be a vector with m elements wehre m is the number of input')
                end
            else
                obj.maxInput = maxInput;
            end
            if(length(maxOutput)~= obj.q)
                % i need to avoid to rise an error for dimension mismatch
                % when i have mutable constraints
                if(isempty(mutable_constr))
                    error('the maxOutput has to be a vector with q elements wehre q is the number of output')
                end
            else
                obj.maxOutput = maxOutput;    
            end
            
            %% Discrete system (when necessary)
            if(~discretized)
                A = eye(obj.n) + delta*A_cont;
                B = obj.delta*B_cont;
                C = C_cont;
            else
                A = A_cont;
                B = B_cont;
                C = C_cont;
            end

            %% Cost Function (here i can manage both scalar and vector cost)
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

            %% managing mutable constraints (this structure will be used inside funcgenerator)
            obj.m_c = mutable_constr;
            if(isempty(mutable_constr))
                obj.m_c_flag = false;
            else
                obj.m_c_flag = true;
                if(obj.m_c.g)
                    obj.m_c.g = "pattern";
                else
                    obj.m_c.g ="";
                end
                if(obj.m_c.w)
                    obj.m_c.w = "pattern";
                else
                    obj.m_c.w ="";
                end
                if(obj.m_c.s)
                    obj.m_c.s = "pattern";
                else
                    obj.m_c.s ="";
                end
            end
            
            %% Construct matrices (it automatically detects fixed or ltv)
            propModelCall             = "CostFunc.propagationModel_regulator_"+ type + "_" + obj.propagationModel + "(obj,A,B,C)";
            [S_bar,T_bar,Q_bar,R_bar] = eval(propModelCall);

            %% Cost function matrices
            costFuncCall      = "CostFunc.regulator_" + obj.costFunc + "(S_bar,T_bar,Q_bar,R_bar)";
            [obj.H,obj.F_tra] = eval(costFuncCall);
            %% Constraints matrices
            
            % g function
            % through obj.m_c.g we now if g is mutable or not 
            constrFuncG_Call = "Constraint.regulator_G_" +obj.m_c.g + "_" + obj.constrG + "(obj,S_bar)";
            obj.G            = eval(constrFuncG_Call);
             % if G is mutable i need to store the current function inside
             % a function handle of the class (needded both for func gen and compute control)
             % and i need to store the variables that G is depending upon
            if(strcmp(obj.m_c.g,"pattern"))
                obj.MutableConstraints_G = str2func(constrFuncG_Call);
                obj.m_c.S_bar = S_bar;
            end
            % S function
            % through obj.m_c.s we now if g is mutable or not 
            constrFuncS_Call = "Constraint.regulator_S_" +obj.m_c.s + "_" + obj.constrS + "(obj,T_bar)";
            obj.S            = eval(constrFuncS_Call);
            % if S is mutable i need to store the current function inside
            % a function handle of the class (needded both for func gen and compute control)
            % and i need to store the variables that G is depending upon
            if(strcmp(obj.m_c.s,"pattern"))
                obj.MutableConstraints_S = str2func(constrFuncS_Call);
                obj.m_c.T_bar = T_bar;
            end
             % w function
             % through obj.m_c.w we now if g is mutable or not 
            constrFuncW_Call = "Constraint.regulator_W_" +obj.m_c.w + "_" + obj.constrW + "(obj)";
            obj.W            = eval(constrFuncW_Call);
            % if W is mutable i need to store the current function inside
            % a function handle of the class (needded both for func gen and compute control)
            % and i need to store the variables that G is depending upon
            if(strcmp(obj.m_c.w,"pattern"))
                obj.MutableConstraints_W = str2func(constrFuncW_Call);
            end
            
            % i copy all the matrix in the sym_* variables in order to pass them to  the function generation method 
            obj.sym_H      = sym(obj.H);                   
            obj.sym_F_tra  = sym(obj.F_tra); 
            obj.sym_G      = sym(obj.G);      % for the function generation it works only if G is not mutable (it works right away both fixed and ltv)
            obj.sym_W      = sym(obj.W);      % for the function generation it works only if W is not mutable (it works right away both fixed and ltv)
            obj.sym_S      = sym(obj.S);      % for the function generation it works only if W is not mutable (it works right away both fixed and ltv)
            
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
                if(strcmp(obj.type,"ltv"))
                    obj.m_c.S_bar_func       = matlabFunction(obj.m_c.S_bar,'vars', {obj.x_0,obj.inner_x_ext});
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
                end 

            else
                if strcmp(obj.type,"ltv")
                    obj.S = matlabFunction(obj.S,'vars', {obj.x_0,obj.inner_x_ext});
                end
            end
            
            
            %% store number of constraints
            % here i compute the number of constraints for each step it is
            % very immportant for the Cpp version of mpc
            obj.N_constr  = size(obj.S,1)/obj.N;
            
        end
        
        % xu_oracle_traj has to contain all the trajectory from the current
        % state to the future one
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
                    S_bar      = obj.m_c.S_bar;
                 else
                     obj.cur_G = obj.G;
                 end
                 if(strcmp(obj.m_c.s,"pattern"))
                    T_bar      = obj.m_c.T_bar;
                 else
                    obj.cur_S  = obj.S;
                 end
                 if(~strcmp(obj.m_w.g,"pattern"))
                    obj.cur_W  = obj.W;
                 end
             end
             tic
             u_star = quadprog(H, x_cur'*F_tra, obj.cur_G, obj.cur_W+obj.cur_S*x_cur);%,[],[],[],[],[],options);
             toc
             tau = u_star(1:obj.m);
             % after each iteration we need to update the mutable constraints 
             if (obj.m_c_flag)
                 % i assume that the pattern are the same for every
                 %  matrix constraint
                 obj.UpdateConstrPattern();
                 if(strcmp(obj.m_w.g,"pattern"))
                    obj.cur_G   = obj.MutableConstraints_G(obj,S_bar);
                 end
                 if(strcmp(obj.m_w.S,"pattern"))
                     obj.cur_S   = obj.MutableConstraints_S(obj,T_bar);
                 end
                 if(strcmp(obj.m_w.w,"pattern"))
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
