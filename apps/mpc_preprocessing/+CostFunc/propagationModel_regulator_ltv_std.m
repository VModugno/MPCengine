
function  [S_bar, T_bar,Q_bar,R_bar]=propagationModel_regulator_ltv_std(obj,A,B,C)
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