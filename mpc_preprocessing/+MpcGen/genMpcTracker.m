classdef genMpcTracker < MpcGen.coreGenerator
    
    properties
        
        orig_n      % state dimension before the exstension with the control input
        variables   % here I define all the variable (to work on the exact meaning of this)
        u_cur       % last input value
       
        maxInput    
        maxOutput     
        %% for logging
        it          % internal iterator to keep track of the data
        Ustar
        Ustar_used
        all_fval
    end
    
    
    methods
        function obj = genMpcTracker(A_cont,B_cont,C_cont,maxInput,maxOutput,delta,N,state_gain,control_cost,type,solver,generate_functions)
                    
            % call super class constructor
            obj = obj@MpcGen.coreGenerator(type,solver,generate_functions);
            
            % problem structure
            obj.type         = type; 
            obj.solver       = solver; 
            obj.problemClass = 'tracker';
            
            % problem dimension
            obj.orig_n      = size(A_cont,1); % state dim
            obj.m           = size(B_cont,2); % control dim
            obj.q           = size(C_cont,1); % output dim
            obj.N           = N;
            obj.delta       = delta;
            % symbolic parameters
            obj.x_0   = sym('x_0',[obj.orig_n,1],'real');
            obj.u_0   = sym('u_0',[obj.m,1],'real');
            obj.ref_0 = sym('ref_0',[obj.N*obj.q,1],'real'); % TOFIX
            
            obj.u_cur = zeros(obj.m,1);
            
            obj.it    = 1;
            
            if(length(maxInput)~= obj.m)
                error('the maxInput has to be a vector with q elements wehre q is the number of output')
            else
                obj.maxInput = maxInput;
            end
            if(length(maxOutput)~= obj.q)
                error('the maxOutput has to be a vector with m elements wehre m is the number of input')
            else
                obj.maxOutput = maxOutput;    
            end
            %% extended system
            A = [eye(obj.orig_n)+obj.delta*A_cont delta*B_cont;zeros(obj.m,obj.orig_n) eye(obj.m,obj.m)];
            B = [obj.delta*B_cont;eye(obj.m,obj.m)];
            C = [C_cont zeros(obj.q,obj.m)];
            obj.n = size(A,1); % extended state dim

            %% Cost Function (for each step)
            Q = state_gain*eye(obj.q);
            R = control_cost*eye(obj.m);

            %% Create ref
%             obj.total_ref = reshape(x_des,length(x_des)*obj.q,1);
%             % extend ref by N to avoid the receding horizon to exceed the
%             % final time
%             last_ref = x_des(:,end);
%             for i=1:N
%                 obj.total_ref = [obj.total_ref;last_ref];
%             end
            %% Create matrix 
            for k = 1:obj.N
                for j = 1:k
                    S_bar(obj.n*(k-1)+(1:obj.n),obj.m*(k-j)+(1:obj.m))     = A^(j-1)*B;
                    S_bar_C(obj.q*(k-1)+(1:obj.q),obj.m*(k-j)+(1:obj.m))           = C*A^(j-1)*B;
                end
                T_bar(obj.n*(k-1)+(1:obj.n),1:obj.n)                   = A^k;
                T_bar_C(obj.q*(k-1)+(1:obj.q),1:obj.n)                         = C*A^k;
                Q_hat(obj.q*(k-1)+(1:obj.q),obj.n*(k-1)+(1:obj.n))         = Q*C;
                Q_bar(obj.n*(k-1)+(1:obj.n),obj.n*(k-1)+(1:obj.n)) = C'*Q*C;
                R_bar(obj.m*(k-1)+(1:obj.m),obj.m*(k-1)+(1:obj.m))                 = R;
            end
            %% Cost function matrices
            obj.H     = (R_bar + S_bar'*Q_bar*S_bar);
            obj.F_tra = [-Q_hat*S_bar ;T_bar'*Q_bar*S_bar];

            %% Constraint matrices
            obj.G     = [ S_bar_C;-S_bar_C; tril(ones(obj.N*obj.m)); -tril(ones(obj.N*obj.m))];
            obj.W     = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput);...
                        -kron(ones(obj.N,1),obj.u_0)+kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.u_0) + kron(ones(obj.N,1),obj.maxInput)];
            obj.W     = matlabFunction(obj.W,'vars', {obj.u_0});
            obj.S     = [-T_bar_C; T_bar_C; zeros(obj.N*obj.m,obj.n); zeros(obj.N*obj.m,obj.n)];
           
            %% i need it for the computation of the controller          
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
            %obj.W     = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput);...
            %            -kron(ones(obj.N,1),obj.u_cur)+kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.u_cur) + kron(ones(obj.N,1),obj.maxInput)];
            W             = obj.W(obj.u_cur);
            % MPC
            %new_F_tra    = [obj.total_ref((it-1)*obj.q + 1:(it-1)*obj.q + obj.N*obj.q) ; x_cur;obj.u_cur]'*obj.F_tra;
            new_F_tra     = [cur_ref; x_cur;obj.u_cur]'*obj.F_tra;
            [u_star,fval] = quadprog(obj.H, new_F_tra, obj.G,W + obj.S*[x_cur;obj.u_cur]);
            
            obj.u_cur     = obj.u_cur + u_star(1: obj.m);
            
            % for debugging
            obj.Ustar{obj.it}        = reshape(u_star,[obj.m,obj.N]);
            obj.Ustar_used(:,obj.it) = obj.u_cur + u_star(1:obj.m);
            obj.all_fval(1,obj.it)   = fval;
            obj.it               = obj.it + 1;
            %
            % control action 
            tau = obj.u_cur; 
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