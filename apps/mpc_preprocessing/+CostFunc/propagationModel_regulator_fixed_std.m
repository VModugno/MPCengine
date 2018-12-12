

function  [S_bar, T_bar,Q_bar,R_bar] = propagationModel_regulator_fixed_std(obj,A,B,C)

    for k = 1:obj.N
        for j = 1:k
            S_bar(obj.q*(k-1)+(1:obj.q),obj.m*(k-j)+(1:obj.m)) = C*A^(j-1)*B;
        end

        T_bar(obj.q*(k-1)+(1:obj.q),1:obj.n) = C*A^k;

        Q_bar(obj.q*(k-1)+(1:obj.q),obj.q*(k-1)+(1:obj.q)) = Q;
        R_bar(obj.m*(k-1)+(1:obj.m),obj.m*(k-1)+(1:obj.m)) = R;
    end
    
end