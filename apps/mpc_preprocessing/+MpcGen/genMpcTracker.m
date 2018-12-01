
%% TODO add mutable constraints here

classdef genMpcTracker < MpcGen.coreGenerator
    
    properties
        
        orig_n      % state dimension before the exstension with the control input
        u_cur       % last input value
       
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
                                     type,solver,generate_functions,discretized,mutable_constr)
                    
            % call super class constructor
            obj = obj@MpcGen.coreGenerator(type,solver,generate_functions,size(A_cont,1),size(B_cont,2),size(C_cont,1),N);
            
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
            %obj.x_0     = sym('x_0',[obj.orig_n,1],'real');
            %obj.u_0     = sym('u_0',[obj.m,1],'real');
            %obj.ref_0   = sym('ref_0',[obj.N*obj.q,1],'real'); 
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
            else
                obj.m_c_flag = true;
            end

            %% Create matrix 
            for k = 1:obj.N
                for j = 1:k
                    S_bar(obj.n*(k-1)+(1:obj.n),obj.m*(k-j)+(1:obj.m))   = A^(j-1)*B;
                    S_bar_C(obj.q*(k-1)+(1:obj.q),obj.m*(k-j)+(1:obj.m)) = C*A^(j-1)*B;
                end
                T_bar(obj.n*(k-1)+(1:obj.n),1:obj.n)                     = A^k;
                T_bar_C(obj.q*(k-1)+(1:obj.q),1:obj.n)                   = C*A^k;
                Q_hat(obj.q*(k-1)+(1:obj.q),obj.n*(k-1)+(1:obj.n))       = Q*C;
                Q_bar(obj.n*(k-1)+(1:obj.n),obj.n*(k-1)+(1:obj.n))       = C'*Q*C;
                R_bar(obj.m*(k-1)+(1:obj.m),obj.m*(k-1)+(1:obj.m))       = R;
            end
            %% Cost function matrices
            obj.H     = (R_bar + S_bar'*Q_bar*S_bar);
            obj.F_tra = [-Q_hat*S_bar ;T_bar'*Q_bar*S_bar];

            %% Constraint matrices
            obj.G     = [ S_bar_C;-S_bar_C; tril(ones(obj.N*obj.m)); -tril(ones(obj.N*obj.m))];
            if (obj.m_c_flag)
                obj.W_numeric = obj.MutableConstraints_W(obj.u_cur);
            else
                obj.W     = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput);...
                            -kron(ones(obj.N,1),obj.u_0)+kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.u_0) + kron(ones(obj.N,1),obj.maxInput)];
                obj.W     = matlabFunction(obj.W,'vars', {obj.u_0});
            end
            obj.S     = [-T_bar_C; T_bar_C; zeros(obj.N*obj.m,obj.n); zeros(obj.N*obj.m,obj.n)];
           
            %% i need it for the computation of the cpp controller function      
            obj.sym_H      = sym(obj.H);                   
            obj.sym_F_tra  = sym(obj.F_tra); 
            obj.sym_G      = sym(obj.G);      
            obj.sym_W      = sym([kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput);...
                             -kron(ones(obj.N,1),obj.u_0)+kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.u_0) + kron(ones(obj.N,1),obj.maxInput)]);
            obj.sym_S      = sym(obj.S);
            
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
            % when we have mutabl constraints
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