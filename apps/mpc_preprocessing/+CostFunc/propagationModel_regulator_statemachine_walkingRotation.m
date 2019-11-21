function  [S_bar_obj,S_bar_constr,T_bar_obj,T_bar_constr,Q_bar,R_bar] = propagationModel_regulator_statemachine_walkingRotation(obj,A,B,C_obj,C_constr,Q,R)

    n                = obj.n(1);
    A                = A{1};
    k_counter_obj    = 0;
    k_counter_constr = 0;
    j_counter_out    = 0;
    
    %% here i build the inner_x_ext for the rotations and the set of symbolical R matrices
    % in order to have a moving window for the rotation as well i need to
    % define N different theta variables one for each time step inside 
    % and than i set the moving window dor theta by changing the theta
    % vector value at each step
    obj.inner_x_ext = [];
    all_R           = cell(obj.N,1);
    % i have always to use the same variables name inside mpcModel 
    for kk = 1:obj.N
        % here i create the symbolic variables
        cur_theta_name = "theta_" + num2str(kk);
        cur_theta = sym(cur_theta_name,[1,1],'real');
        % current rotation matrix (already transposed)
        R_t = [cos(cur_theta), sin(cur_theta); -sin(cur_theta), cos(cur_theta)];
        % i store the resulting value inside all A and all B
        all_R{kk} = R_t;
        obj.inner_x_ext = [obj.inner_x_ext;cur_theta];
    end

    %% here i build the matrix for the model propagation
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
            % the cost function is not directly changed by the rotation
            % matrix
            S_bar_obj(k_counter_obj + (1:q_obj),j_counter+(1:m))          = C_k_obj*A^(k-j)*B_j;
            % the rotation impact only the matrices involved in the
            % construction of the constraints 
            S_bar_constr(k_counter_constr + (1:q_constr),j_counter+(1:m)) = all_R{k-j + 1}*C_k_constr*A^(k-j)*B_j;
            j_counter = j_counter + m;
        end

        T_bar_obj(k_counter_obj + (1:q_obj),1:n)                   = C_k_obj*A^k;
        T_bar_constr(k_counter_constr + (1:q_constr),1:n)          = all_R{1}*C_k_constr*A^k;

        Q_bar(k_counter_obj + (1:q_obj),k_counter_obj + (1:q_obj)) = Q{k_state};
        R_bar(j_counter_out+(1:m),j_counter_out+(1:m))             = R{k_state};
        
        k_counter_obj    = k_counter_obj + q_obj;
        k_counter_constr = k_counter_constr + q_constr;
        j_counter_out    = j_counter_out + obj.m(k_state);
        
    end
    
end