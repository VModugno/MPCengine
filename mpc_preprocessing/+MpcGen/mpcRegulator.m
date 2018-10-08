classdef mpcRegulator < MpcGen.coreGenerator


    properties
        
        variables   % here i define all the variable 
        n           % state space dim
        m           % control space dim 
        q           % output space dim
        
        maxInput    
        maxOutput   
        delta       % sampling time
        N           % widht of prediction window
        n_v         % total number of constraints given N and the constraints in one step
        n_c         % total number of decision variables (control space dim X N) 
        
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
        function obj = mpcRegulator(A_cont,B_cont,C_cont,maxInput,maxOutput,delta,N,state_gain,control_cost,type,solver)
            
            % call super class constructor
            obj = obj@MpcGen.coreGenerator(type,solver);
            
            obj.n     = size(A_cont,1);
            obj.m     = size(B_cont,2);
            obj.q     = size(C_cont,1);
            obj.N     = N;
            obj.delta = delta; 
            obj.x_0   = sym('x_0',obj.n);
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
            H = 2*(R_bar + S_bar'*Q_bar*S_bar);
            F_tra = 2*T_bar'*Q_bar*S_bar;

            %% Constraint matrices
            G    = [S_bar; -S_bar; eye(obj.N*obj.m); -eye(obj.N*obj.m)];
            W    = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.maxInput)];
            S    = [-T_bar; T_bar; zeros(obj.N*obj.m,obj.n); zeros(obj.N*obj.m,obj.n)];
            
            obj.type   = type; 
            obj.solver = solver; 
            
            obj.sym_H      = sym(H);                   
            obj.sym_F_tra  = sym(F_tra); 
            obj.sym_G      = sym(G);      
            obj.sym_W      = sym(W);
            obj.sym_S      = sym(S);
        end
        
        
        function GenFunction(obj) 
          
             GenFunction@MpcGen.coreGenerator(obj)
             
       end
        
    end


end
