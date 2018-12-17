
%% TODO add mutable constraints here

classdef genMpcTracker < MpcGen.coreGenerator
    
    properties
        
        orig_n      % state dimension before the exstension with the control input
        u_cur       % last input value (numerical version of u_prev)
       
        maxInput    
        maxOutput     
        %% for logging
        it          % internal iterator to keep track of the data
        Ustar
        Ustar_used
        all_fval
        W_numeric  % i need this variable becasue for tracker i have defined function to compute W (obj.W cannot be used in the same way of regulator)
    end
    
    
    methods
        function obj = genMpcTracker(A_cont,B_cont,C_cont,maxInput,maxOutput,delta,N,state_gain,control_cost,...
                                     type,solver,generate_functions,discretized,mutable_constr,function_list)
                    
            % call super class constructor
            obj = obj@MpcGen.coreGenerator(type,solver,generate_functions,size(A_cont,1),size(B_cont,2),size(C_cont,1),N,function_list);
            
            % problem structure
            obj.type         = type; 
            obj.solver       = solver; 
            obj.problemClass = 'tracker';
            
            % problem dimension
            obj.orig_n       = size(A_cont,1); % state dim
            obj.m            = size(B_cont,2); % control dim
            obj.q            = size(C_cont,1); % output dim
            obj.N            = N;
            obj.delta        = delta;
            % symbolic parameters
            % when we do not have external varialbes to optimize  we assign a dimension of one just to allow
            % matlab to provide the right functions signature
            % i need to set a dimension of 2 here in order to force the
            % ccode function to generate a pointer in the signature
            obj.outer_x    = sym('outer_x',[2,1],'real'); 
            obj.extern_var = "false";
            obj.extern_dim = 0;
            
            obj.u_cur = zeros(obj.m,1);
            
            obj.it    = 1;
            
            if(length(maxInput)~= obj.m)
                if(isempty(mutable_constr))
                    error('the maxInput has to be a vector with q elements wehre q is the number of output')
                end
            else
                obj.maxInput = maxInput;
            end
            if(length(maxOutput)~= obj.q)
                if(isempty(mutable_constr))
                    error('the maxOutput has to be a vector with m elements wehre m is the number of input')
                end
            else
                obj.maxOutput = maxOutput;    
            end
            %% extended system and discretize when necessary
            if(~discretized)
                % discretization and exstension
                A = [eye(obj.orig_n)+obj.delta*A_cont delta*B_cont;zeros(obj.m,obj.orig_n) eye(obj.m,obj.m)];
                B = [obj.delta*B_cont;eye(obj.m,obj.m)];
                C = [C_cont zeros(obj.q,obj.m)];
                obj.n = size(A,1); % extended state dim
            else
                % only exstension
                A = [A_cont B_cont;zeros(obj.m,obj.orig_n) eye(obj.m,obj.m)];
                B = [B_cont;eye(obj.m,obj.m)];
                C = [C_cont zeros(obj.q,obj.m)];
                obj.n = size(A,1); % extended state dim
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
            
            %% managing mutable constraints
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
             if(strcmp(obj.type,"ltv"))
                [A,B]=ComputeMatricesLTV(obj,A,B);
            else
                obj.inner_x_ext = []; % empty vector
             end
            %% Construct matrices (it automatically detects fixed or ltv)
            propModelCall             = "CostFunc.propagationModel_tracker_"+ type + "_" + obj.propagationModel + "(obj,A,B,C,Q,R)";
            [S_bar,S_bar_C,T_bar,T_bar_C,Q_bar,R_bar] = eval(propModelCall);
            %% Cost function matrices
            costFuncCall      = "CostFunc.tracker_" + obj.costFunc + "(S_bar,T_bar,Q_bar,R_bar)";
            [obj.H,obj.F_tra] = eval(costFuncCall);

            %% Constraint matrices
            %% Constraints matrices
            
            % g function
            % through obj.m_c.g we know if g is mutable or not 
            constrFuncG_Call = "Constraint.tracker_G_" +obj.m_c.g + "_" + obj.constrG + "(obj,S_bar_C)";
            obj.G            = eval(constrFuncG_Call);
             % if G is mutable i need to store the current function inside
             % a function handle of the class (needded both for func gen and compute control)
             % and i need to store the variables that G is depending upon
            if(strcmp(obj.m_c.g,"pattern"))
                str2funcCall             = "Constraint.tracker_G_" +obj.m_c.g + "_" + obj.constrG + "(obj,S_bar_C)";
                obj.MutableConstraints_G = str2func(str2funcCall);
                obj.m_c.S_bar_C          = S_bar_C;
            end
            % S function
            % through obj.m_c.s we know if g is mutable or not 
            constrFuncS_Call = "Constraint.tracker_S_" +obj.m_c.s + "_" + obj.constrS + "(obj,T_bar_C)";
            obj.S            = eval(constrFuncS_Call);
            % if S is mutable i need to store the current function inside
            % a function handle of the class (needded both for func gen and compute control)
            % and i need to store the variables that G is depending upon
            if(strcmp(obj.m_c.s,"pattern"))
                str2funcCall             = "Constraint.tracker_S_" +obj.m_c.s + "_" + obj.constrS;   
                obj.MutableConstraints_S = str2func(str2funcCall);
                obj.m_c.T_bar_C          = T_bar_C;
            end
             % w function
             % through obj.m_c.w we know if g is mutable or not 
            constrFuncW_Call = "Constraint.tracker_W_" +obj.m_c.w + "_" + obj.constrW + "(obj,obj.u_prev)";
            obj.W            = eval(constrFuncW_Call);
            % if W is mutable i need to store the current function inside
            % a function handle of the class (needded both for func gen and compute control)
            % and i need to store the variables that G is depending upon
            if(strcmp(obj.m_c.w,"pattern"))
                str2funcCall             = "Constraint.tracker_W_" +obj.m_c.w + "_" + obj.constrW;
                obj.MutableConstraints_W = str2func(str2funcCall);
            end
            
%             if (obj.m_c_flag)
%                 obj.W_numeric = obj.MutableConstraints_W(obj.u_cur);
%             else
%                 obj.W     = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput);...
%                             -kron(ones(obj.N,1),obj.u_prev)+kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.u_prev) + kron(ones(obj.N,1),obj.maxInput)];
%                 obj.W     = matlabFunction(obj.W,'vars', {obj.u_prev});
%             end
            
           
            %% i copy all the matrix in the sym_* variables in order to pass them to the function generation method   
            obj.sym_H      = sym(obj.H);                   
            obj.sym_F_tra  = sym(obj.F_tra); 
            obj.sym_G      = sym(obj.G);      
            %obj.sym_W      = sym([kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput);...
            %                 -kron(ones(obj.N,1),obj.u_prev)+kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.u_prev) + kron(ones(obj.N,1),obj.maxInput)]);
            obj.sym_W      = sym(obj.W);
            obj.sym_S      = sym(obj.S);
            
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
                if strcmp(obj.type,"ltv")
                    obj.S = matlabFunction(obj.S,'vars', {obj.x_0,obj.inner_x_ext});
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
            obj.N_constr  = size(obj.S,1)/obj.N;
            
            
        end
        
        function tau = ComputeControl(obj,x_cur,cur_ref)
            % i do not update here the W the W for mutable constraints case
            if (~obj.m_c_flag)
                obj.W_numeric = obj.W(obj.u_cur);
            end
            new_F_tra = [cur_ref; x_cur;obj.u_cur]'*obj.F_tra;
            
            %% debug
            %inner_x = [cur_ref; x_cur;obj.u_cur];
            %A_      = obj.G';
            %A_      = A_(:);
            %g_      = ([cur_ref; x_cur;obj.u_cur]'*obj.F_tra)';
            %H_      = obj.H';
            %H_      = H_(:);
            %ub_     = W + obj.S*[x_cur;obj.u_cur];
           
            [u_star,fval,exitflag,output,lambda] = quadprog(obj.H, new_F_tra, obj.G,obj.W_numeric + obj.S*[x_cur;obj.u_cur]);
            % new control
            obj.u_cur     = obj.u_cur + u_star(1: obj.m);
            % after updating u_cur i can update W for the next iteration 
            % when we have mutable constraints
            if (obj.m_c_flag)
                obj.UpdateConstrPattern();
                obj.W_numeric = obj.MutableConstraints_W(obj.u_cur);
            end
            
            
            % for debugging
            obj.Ustar{obj.it}        = reshape(u_star,[obj.m,obj.N]);
            obj.Ustar_used(:,obj.it) = obj.u_cur + u_star(1:obj.m);
            obj.all_fval(1,obj.it)   = fval;
            obj.it                   = obj.it + 1;
            %
            % control action 
            tau = obj.u_cur; 
        end
        function W = MutableConstraints_W(obj,u_cur)
            part_W = zeros(obj.N*obj.q,1);
            for jj = 1:obj.m_c.N_state
                part_W  = part_W + kron(obj.m_c.const_pattern(:,jj), obj.m_c.bounds(:,jj));
            end
            W = [part_W;part_W];
            % adding constraints about input
            W =[W;
               -kron(ones(obj.N,1),u_cur)+kron(ones(obj.N,1),obj.maxInput);
                kron(ones(obj.N,1),u_cur) + kron(ones(obj.N,1),obj.maxInput)]; 
        end
        
%         function PlotGraph(obj)
%             figure
%             plot(obj.Ustar_used');
%             grid;
%             for i = 1:size(obj.Ustar_used,1)
%                 leg_text(1,i) = 'tau_'+ num2str(i) + ' (MPC)';
%             end
%             legend(leg_text);
%         end
        
    end
    
end