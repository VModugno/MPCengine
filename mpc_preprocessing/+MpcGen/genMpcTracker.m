classdef genMpcTracker < Ctrl.AbstractController
    
    properties
        A
        B
        C
        Q
        R
        S_bar
        S_bar_C
        T_bar
        T_bar_C
        Q_hat
        Q_bar
        R_bar
        H
        F_tra
        G
        n
        m
        q
        n_ext
        maxInput
        maxOutput
        delta
        N
        total_ref
        u_cur      
        %% for logging
        Ustar
        Ustar_used
        all_fval
    end
    
    
    methods
        function obj = genMpcTracker(A_cont,B_cont,C_cont,x_des,maxInput,maxOutput,delta,N,state_gain,control_cost)
            obj.n = size(A_cont,1); % state dim
            obj.m = size(B_cont,2); % control dim
            obj.q = size(C_cont,1); % output dim
            obj.N = N;
            obj.delta = delta;
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
            obj.A = [eye(obj.n)+obj.delta*A_cont delta*B_cont;zeros(obj.m,obj.n) eye(obj.m,obj.m)];
            obj.B = [obj.delta*B_cont;eye(obj.m,obj.m)];
            obj.C = [C_cont zeros(obj.q,obj.m)];
            obj.n_ext = size(obj.A,1); % extended state dim

            %% Cost Function (for each step)
            obj.Q = state_gain*eye(obj.q);
            obj.R = control_cost*eye(obj.m);

            %% Create ref
            obj.total_ref = reshape(x_des,length(x_des)*obj.q,1);
            % extend ref by N to avoid the preceeding horizon to exceed the
            % final time
            last_ref = x_des(:,end);
            for i=1:N
                obj.total_ref = [obj.total_ref;last_ref];
            end
            %% Create matrix 
            for k = 1:obj.N
                for j = 1:k
                    obj.S_bar(obj.n_ext*(k-1)+(1:obj.n_ext),obj.m*(k-j)+(1:obj.m))     = obj.A^(j-1)*obj.B;
                    obj.S_bar_C(obj.q*(k-1)+(1:obj.q),obj.m*(k-j)+(1:obj.m))           = obj.C*obj.A^(j-1)*obj.B;
                end
                obj.T_bar(obj.n_ext*(k-1)+(1:obj.n_ext),1:obj.n_ext)                   = obj.A^k;
                obj.T_bar_C(obj.q*(k-1)+(1:obj.q),1:obj.n_ext)                         = obj.C*obj.A^k;
                obj.Q_hat(obj.q*(k-1)+(1:obj.q),obj.n_ext*(k-1)+(1:obj.n_ext))         = obj.Q*obj.C;
                obj.Q_bar(obj.n_ext*(k-1)+(1:obj.n_ext),obj.n_ext*(k-1)+(1:obj.n_ext)) = obj.C'*obj.Q*obj.C;
                obj.R_bar(obj.m*(k-1)+(1:obj.m),obj.m*(k-1)+(1:obj.m))                 = obj.R;
            end
            %% Cost function matrices
            obj.H     = (obj.R_bar + obj.S_bar'*obj.Q_bar*obj.S_bar);
            obj.F_tra = [-obj.Q_hat*obj.S_bar ;obj.T_bar'*obj.Q_bar*obj.S_bar];

            %% Constraint matrices
            obj.G     = [obj.S_bar_C; -obj.S_bar_C; tril(ones(obj.N*obj.m)); -tril(ones(obj.N*obj.m))];
            %% i need it for the computation of the controller
            obj.u_cur = zeros(obj.m,1);
        end
        
        function tau = ComputeControl(obj,x_cur,it)
            W     = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput);...
                    -kron(ones(obj.N,1),obj.u_cur)+kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.u_cur) + kron(ones(obj.N,1),obj.maxInput)];
            S     = [-obj.T_bar_C; obj.T_bar_C; zeros(obj.N*obj.m,obj.n_ext); zeros(obj.N*obj.m,obj.n_ext)];

            % MPC
            new_F_tra     = [obj.total_ref((it-1)*obj.q + 1:(it-1)*obj.q + obj.N*obj.q) ; x_cur;obj.u_cur]'*obj.F_tra;
            [u_star,fval] = quadprog(obj.H, new_F_tra, obj.G, W+S*[x_cur;obj.u_cur]);
            
            obj.u_cur     = obj.u_cur + u_star(1: obj.m);
            
            % for debugging
            obj.Ustar{it}        = reshape(u_star,[obj.m,obj.N]);
            obj.Ustar_used(:,it) = obj.u_cur + u_star(1:obj.m);
            obj.all_fval(1,it)   = fval;
            %
            % control action 
            tau = obj.u_cur; 
        end
        
        function PlotGraph(obj)
            figure
            plot(obj.Ustar_used');
            grid;
            for i = 1:size(obj.Ustar_used,1)
                leg_text(1,i) = 'tau_'+ num2str(i) + ' (MPC)';
            end
            legend(leg_text);
        end
        
    end
    
end