classdef genMpcRegulator < MpcGen.coreGenerator


    properties
          
        maxInput    
        maxOutput 
        inner_x_ext
        
       
        
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

            %% managing mutable constraints
            obj.m_c = mutable_constr;
            if(isempty(mutable_constr))
                obj.m_c_flag = false;
            else
                obj.m_c_flag = true;
            end
            
            %% Construct matrices
            %% fixed
            if(strcmp(obj.type,"fixed"))
                for k = 1:obj.N
                    for j = 1:k
                        S_bar(obj.q*(k-1)+(1:obj.q),obj.m*(k-j)+(1:obj.m)) = C*A^(j-1)*B;
                    end

                    T_bar(obj.q*(k-1)+(1:obj.q),1:obj.n) = C*A^k;

                    Q_bar(obj.q*(k-1)+(1:obj.q),obj.q*(k-1)+(1:obj.q)) = Q;
                    R_bar(obj.m*(k-1)+(1:obj.m),obj.m*(k-1)+(1:obj.m)) = R;
                end
            %% LTV    
            elseif(strcmp(obj.type,"ltv"))
                
                % here i build all the A and B matrix with their
                % dependancy (u_k,x_k) and the vector of variables
                % x_inner_extended
                obj.inner_x_ext = [];
                all_A           = cell(obj.N,1);
                all_B           = cell(obj.N,1);
                % i have always to use the same variables name inside mpcModel 
                x               = sym('x',[obj.n,1],'real');
                u               = sym('u',[obj.m,1],'real');
                for kk = 1:obj.N
                    % here i create the symbolic variables
                    cur_x_name = "x_" + num2str(kk-1);
                    cur_u_name = "u_" + num2str(kk-1);
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
                    if(kk==1)
                         obj.inner_x_ext = [obj.inner_x_ext;cur_u];
                    else
                         % the order which i store this variables is gonna
                         % be the orders that i have to observe when i pass
                         % the variables to the function
                         obj.inner_x_ext = [obj.inner_x_ext;cur_x;cur_u];
                    end
                    
                end
                % constructing matrices
                
%                 % only for debugging with cart pole model!
%                 syms a0 a1 a2 a3 a4 a5 a6 a7 a8 a9
%                 A0 = diag(a0*ones(obj.n,1)); A1 = diag(a1*ones(obj.n,1)); A2 = diag(a2*ones(obj.n,1)); 
%                 A3 = diag(a3*ones(obj.n,1));A4 = diag(a4*ones(obj.n,1)); 
%                 A5 = diag(a5*ones(obj.n,1));A6 = diag(a6*ones(obj.n,1)); A7 = diag(a7*ones(obj.n,1)); 
%                 A8 = diag(a8*ones(obj.n,1));A9 = diag(a9*ones(obj.n,1));
%                 all_A = {A0 A1 A2 A3 A4 A5 A6 A7 A8 A9};
%                 
%                 syms b0 b1 b2 b3 b4 b5 b6 b7 b8 b9
%                 B0 = b0*ones(obj.n,1);B1 = b1*ones(obj.n,1);B2 = b2*ones(obj.n,1);B3 = b3*ones(obj.n,1);B4 = b4*ones(obj.n,1); B5 = b5*ones(obj.n,1);
%                 B6 = b6*ones(obj.n,1);B7 = b7*ones(obj.n,1);B8 = b8*ones(obj.n,1);B9 = b9*ones(obj.n,1);
%                 all_B = {B0 B1 B2 B3 B4 B5 B6 B7 B8 B9};
%                 % only for debugging
                
                ca = eye(obj.n);
                
                for k = 1:obj.N          
                   
                    % building the matrix that contains the system
                    % evolution (look Bemporad slides)
                    A_prod = eye(obj.n);
                    for j = 1:k
                        if(j>1)
                            A_prod = A_prod*all_A{k + 2 - j};
                        end
                        S_bar(obj.q*(k-1)+(1:obj.q),obj.m*(k-j)+(1:obj.m)) = C*A_prod*all_B{k+1-j};
                    end
                    % building the term that multiply the x0 (look Bemporad slides)
                    ca = all_A{k}*ca;
                    T_bar(obj.q*(k-1)+(1:obj.q),1:obj.n) = C*ca;
                    

                    Q_bar(obj.q*(k-1)+(1:obj.q),obj.q*(k-1)+(1:obj.q)) = Q;
                    R_bar(obj.m*(k-1)+(1:obj.m),obj.m*(k-1)+(1:obj.m)) = R;
                end
                
            end

            %% Cost function matrices
            obj.H = 2*(R_bar + S_bar'*Q_bar*S_bar);
            obj.F_tra = 2*T_bar'*Q_bar*S_bar;

            %% Constraint matrices
            obj.G    = [S_bar; -S_bar; eye(obj.N*obj.m); -eye(obj.N*obj.m)];
            % if mutable_constr_flag is true i need to build the W matrix
            % according to the foot_pattern
            if (obj.m_c_flag)    
               dummy_var = 0; 
               obj.W     = obj.MutableConstraints_W(dummy_var);
            else
               obj.W     = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.maxInput)];
            end
            obj.S        = [-T_bar; T_bar; zeros(obj.N*obj.m,obj.n); zeros(obj.N*obj.m,obj.n)];
             
            obj.sym_H      = sym(obj.H);                   
            obj.sym_F_tra  = sym(obj.F_tra); 
            obj.sym_G      = sym(obj.G);      
            obj.sym_W      = sym(obj.W);
            obj.sym_S      = sym(obj.S);
            
            % if the matrix as been computed for LTV i need to transform
            % them into matlab function to use them inside matlab for
            % compute control 
            if(strcmp(obj.type,"ltv"))
                obj.H     = matlabFunction(obj.H,'vars', {obj.x_0,obj.inner_x_ext});
                obj.F_tra = matlabFunction(obj.F_tra,'vars', {obj.x_0,obj.inner_x_ext});
                obj.G     = matlabFunction(obj.G,'vars', {obj.x_0,obj.inner_x_ext});
                obj.S     = matlabFunction(obj.S,'vars', {obj.x_0,obj.inner_x_ext});
            end
            
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
                 G     = obj.G(xu_oracle_trajectory);
                 S     = obj.S(xu_oracle_trajectory);
             elseif(strcmp(obj.type,"fixed"))
                 H     = obj.H;
                 F_tra = obj.F_tra;
                 G     = obj.G;
                 S     = obj.S;    
             end
            
             tic
             u_star = quadprog(H, x_cur'*F_tra, G, obj.W+S*x_cur);%,[],[],[],[],[],options);
             toc
             tau = u_star(1:obj.m);
             % W has to be update after each new control signal has been computed if i have mutable 
             if (obj.m_c_flag)
                 dummy_var = 0;
                 obj.UpdateConstrPattern();
                 obj.W   = obj.MutableConstraints_W(dummy_var);
             end
        end
        
        function W = MutableConstraints_W(obj,dummy_var)
            part_W = zeros(obj.N*obj.q,1);
            for jj = 1:obj.m_c.N_state
                part_W  = part_W + kron(obj.m_c.const_pattern(:,jj), obj.m_c.bounds(:,jj));
            end
            W = [part_W;part_W];


            % adding constraints about input
            W =[W;
               kron(ones(obj.N,1), obj.maxInput);
               kron(ones(obj.N,1), obj.maxInput)]; 
        end
        
        function GenFunctions(obj) 
             GenFunctions@MpcGen.coreGenerator(obj)  
        end
       
        function GenEnvParametersFile()
            [pNode]=GenEnvParametersFile@MpcGen.coreGenerator(obj);
        end
        
    end


end
