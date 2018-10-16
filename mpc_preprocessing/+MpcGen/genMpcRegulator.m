classdef genMpcRegulator < MpcGen.coreGenerator


    properties
        
        variables   % here I define all the variable (to work on the exact meaning of this)
        n           % state space dim
        m           % control space dim 
        q           % output space dim 
        delta       % sampling time
        N           % widht of prediction window
        N_constr    % number of constraints
        
        maxInput    
        maxOutput 
        
        % they are in the  superclass
        %type        % ltv or fixed 
        %solver      % solver target
        %sym_H       %              
        %sym_F_tra   %
        %sym_G       %
        %Sym_W       %
        %sym_S       %
        %x_0         % sym vector to update current state
        
    end



    methods
        function obj = genMpcRegulator(A_cont,B_cont,C_cont,maxInput,maxOutput,delta,N,state_gain,control_cost,type,solver)
            
            % call super class constructor
            obj = obj@MpcGen.coreGenerator(type,solver);
            obj.n     = size(A_cont,1);
            obj.m     = size(B_cont,2);
            obj.q     = size(C_cont,1);
            obj.N     = N;
            obj.delta = delta; 
            obj.x_0   = sym('x_0',[obj.n,1],'real');
            if(length(maxInput)~= obj.m)
                error('the maxInput has to be a vector with m elements wehre m is the number of input')
            else
                obj.maxInput = maxInput;
            end
            if(length(maxOutput)~= obj.q)
                error('the maxOutput has to be a vector with q elements wehre q is the number of output')
            else
                obj.maxOutput = maxOutput;    
            end
            

            %% Discrete system
            A = eye(obj.n) + delta*A_cont;
            B = obj.delta*B_cont;
            C = C_cont;

            %% Cost Function
            Q = state_gain*eye(obj.q);
            R = control_cost*eye(obj.m);

            %% Construct matrices
            for k = 1:obj.N
                for j = 1:k
                    S_bar(obj.q*(k-1)+(1:obj.q),obj.m*(k-j)+(1:obj.m)) = C*A^(j-1)*B;
                end

                T_bar(obj.q*(k-1)+(1:obj.q),1:obj.n) = C*A^k;

                Q_bar(obj.q*(k-1)+(1:obj.q),obj.q*(k-1)+(1:obj.q)) = Q;
                R_bar(obj.m*(k-1)+(1:obj.m),obj.m*(k-1)+(1:obj.m)) = R;
            end

            %% Cost function matrices
            obj.H = 2*(R_bar + S_bar'*Q_bar*S_bar);
            obj.F_tra = 2*T_bar'*Q_bar*S_bar;

            %% Constraint matrices
            obj.G    = [S_bar; -S_bar; eye(obj.N*obj.m); -eye(obj.N*obj.m)];
            obj.W    = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.maxInput)];
            obj.S    = [-T_bar; T_bar; zeros(obj.N*obj.m,obj.n); zeros(obj.N*obj.m,obj.n)];
            
            obj.type   = type; 
            obj.solver = solver; 
            
            obj.sym_H      = sym(H);                   
            obj.sym_F_tra  = sym(F_tra); 
            obj.sym_G      = sym(G);      
            obj.sym_W      = sym(W);
            obj.sym_S      = sym(S);
        end
        
        
         function tau = ComputeControl(obj,x_cur)
            % why here we repeat the computation of S at each time
             %S = [-obj.T_bar; obj.T_bar; zeros(obj.N*obj.m,obj.n); zeros(obj.N*obj.m,obj.n)];
             u_star = quadprog(obj.H, x_cur'*obj.F_tra, obj.G, obj.W+S*x_cur);
             tau = u_star(1:obj.m);
         end
        
        
        function GenFunctions(obj) 
             GenFunctions@MpcGen.coreGenerator(obj)  
       end
        
    end


end
