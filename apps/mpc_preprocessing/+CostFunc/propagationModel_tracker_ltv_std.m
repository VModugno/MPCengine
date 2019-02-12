

function  [S_bar,S_bar_C,T_bar,T_bar_C,Q_hat,Q_bar,R_bar] = propagationModel_tracker_ltv_std(obj,all_A,all_B,C,Q,R)


    ca = eye(obj.n);

    for k = 1:obj.N          
        tic
        % building the matrix that contains the system
        % evolution (look Bemporad slides)
        A_prod = eye(obj.n);
        for j = 1:k
            if(j>1)
                A_prod = A_prod*all_A{k + 2 - j};
            end
            temp_0 = A_prod*all_B{k+1-j};
            S_bar(obj.n*(k-1)+(1:obj.n),obj.m*(k-j)+(1:obj.m))   = temp_0;
            S_bar_C(obj.q*(k-1)+(1:obj.q),obj.m*(k-j)+(1:obj.m)) = C*temp_0;
        end
        % building the term that multiply the x0 (look Bemporad slides)
        ca = all_A{k}*ca;
        T_bar(obj.n*(k-1)+(1:obj.n),1:obj.n)   = ca;
        T_bar_C(obj.q*(k-1)+(1:obj.q),1:obj.n) = C*ca;

        temp_1 = Q*C;
        Q_hat(obj.q*(k-1)+(1:obj.q),obj.n*(k-1)+(1:obj.n)) = temp_1;
        Q_bar(obj.n*(k-1)+(1:obj.n),obj.n*(k-1)+(1:obj.n)) = C'*temp_1;
        R_bar(obj.m*(k-1)+(1:obj.m),obj.m*(k-1)+(1:obj.m)) = R;
        toc
    end
%     for k = 1:obj.N
%         for j = 1:k
%             S_bar(obj.n*(k-1)+(1:obj.n),obj.m*(k-j)+(1:obj.m))   = A^(j-1)*B;
%             S_bar_C(obj.q*(k-1)+(1:obj.q),obj.m*(k-j)+(1:obj.m)) = C*A^(j-1)*B;
%         end
%         T_bar(obj.n*(k-1)+(1:obj.n),1:obj.n)                     = A^k;
%         T_bar_C(obj.q*(k-1)+(1:obj.q),1:obj.n)                   = C*A^k;
%         Q_hat(obj.q*(k-1)+(1:obj.q),obj.n*(k-1)+(1:obj.n))       = Q*C;
%         Q_bar(obj.n*(k-1)+(1:obj.n),obj.n*(k-1)+(1:obj.n))       = C'*Q*C;
%         R_bar(obj.m*(k-1)+(1:obj.m),obj.m*(k-1)+(1:obj.m))       = R;
%     end
end