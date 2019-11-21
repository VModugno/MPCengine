function  [S_bar_obj,S_bar_constr,T_bar_obj,T_bar_constr,Q_bar,R_bar] = propagationModel_regulator_statemachine_std(obj,A,B,C_obj,C_constr,Q,R)

    n                = obj.n(1);
    A                = A{1};
    k_counter_obj    = 0;
    k_counter_constr = 0;
    j_counter_out    = 0;
    for k = 1:obj.N % k = rows n (rows of C)
        
        k_state = obj.state_machine.state_pattern(k);
                
        C_k_obj    = C_obj{k_state};
        C_k_constr = C_constr{k_state};
        
        q_obj    = obj.q_obj(k_state);
        q_constr = obj.q_constr(k_state);
        
        j_counter = 0;
        for j = 1:k %  j = columns (columns of B)
            j_state = obj.state_machine.state_pattern(j);
            B_j     = B{j_state};
            m       = obj.m(j_state);
            S_bar_obj(k_counter_obj + (1:q_obj),j_counter+(1:m))          = C_k_obj*A^(k-j)*B_j;
            S_bar_constr(k_counter_constr + (1:q_constr),j_counter+(1:m)) = C_k_constr*A^(k-j)*B_j;
            j_counter = j_counter + m;
        end

        T_bar_obj(k_counter_obj + (1:q_obj),1:n)                   = C_k_obj*A^k;
        T_bar_constr(k_counter_constr + (1:q_constr),1:n)          = C_k_constr*A^k;

        Q_bar(k_counter_obj + (1:q_obj),k_counter_obj + (1:q_obj)) = Q{k_state};
        R_bar(j_counter_out+(1:m),j_counter_out+(1:m))             = R{k_state};
        
        k_counter_obj    = k_counter_obj + q_obj;
        k_counter_constr = k_counter_constr + q_constr;
        j_counter_out    = j_counter_out + obj.m(k_state);
        
    end
    
end