classdef mpcRegulator < Ctrl.AbstractController


    properties
        A
        B
        C
        Q
        R
        S_bar
        T_bar
        Q_bar
        R_bar
        H
        F_tra
        W
        G
        n
        m
        q
        maxInput
        maxOutput
        delta
        N
    end



    methods
        function obj = mpcRegulator(A_cont,B_cont,C_cont,maxInput,maxOutput,delta,N,state_gain,control_cost)
            obj.n     = size(A_cont,1);
            obj.m     = size(B_cont,2);
            obj.q     = size(C_cont,1);
            obj.N     = N;
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
            
            

            %% Discrete system
            obj.A = eye(obj.n) + delta*A_cont;
            obj.B = obj.delta*B_cont;
            obj.C = C_cont;

            %% Cost Function
            obj.Q = state_gain*eye(obj.q);
            obj.R = control_cost*eye(obj.m);

            %% Construct matrices
            for k = 1:obj.N
                for j = 1:k
                    obj.S_bar(obj.q*(k-1)+(1:obj.q),obj.m*(k-j)+(1:obj.m)) = obj.C*obj.A^(j-1)*obj.B;
                end

                obj.T_bar(obj.q*(k-1)+(1:obj.q),1:obj.n) = obj.C*obj.A^k;

                obj.Q_bar(obj.q*(k-1)+(1:obj.q),obj.q*(k-1)+(1:obj.q)) = obj.Q;
                obj.R_bar(obj.m*(k-1)+(1:obj.m),obj.m*(k-1)+(1:obj.m)) = obj.R;
            end

            %% Cost function matrices
            obj.H = 2*(obj.R_bar + obj.S_bar'*obj.Q_bar*obj.S_bar);
            obj.F_tra = 2*obj.T_bar'*obj.Q_bar*obj.S_bar;

            %% Constraint matrices
            obj.G = [obj.S_bar; -obj.S_bar; eye(obj.N*obj.m); -eye(obj.N*obj.m)];
            obj.W = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.maxInput)];
        end
        
        function tau = ComputeControl(obj,x_cur)
            % why here we repeat the computation of S at each time
             S = [-obj.T_bar; obj.T_bar; zeros(obj.N*obj.m,obj.n); zeros(obj.N*obj.m,obj.n)];
             u_star = quadprog(obj.H, x_cur'*obj.F_tra, obj.G, obj.W+S*x_cur);
             tau = u_star(1:obj.m);
        end
        
        function PlotGraph(obj)
            
        end
        
    end


end
