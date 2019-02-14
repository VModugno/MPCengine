function W = regulator_W_statemachine_pattern_walking(obj)

    input_counter = 0; 
    for k = 1:obj.N % k = rows n (rows of C)
        k_state = obj.state_machine.state_pattern(k);
        m       = obj.m(k_state);
        % this way of treating the footstep indexing works only with two
        % cases in the mutable constraints (in this left and right)
        left_or_right = (mod(obj.m_c.footstep_pattern(k),2) == 0) + 1; 
        
        W_input_max(input_counter+(1:m),1) = obj.m_c.boundsInput.max{k_state}(:,left_or_right);
        W_input_min(input_counter+(1:m),1) = obj.m_c.boundsInput.min{k_state}(:,left_or_right);
        input_counter = input_counter + m;
    end
    output_counter = 0; 
    for k = 1:obj.N % k = rows n (rows of C)
        k_state = obj.state_machine.state_pattern(k);
        q       = obj.q_constr(k_state);
        % this way of treating the footstep indexing works only with two
        % cases in the mutable constraints (in this left and right)
        left_or_right = (mod(obj.m_c.footstep_pattern(k),2) == 0) + 1; 
        
        W_output_max(output_counter+(1:q),1) = obj.m_c.boundsOutput.max{k_state}(:,left_or_right);
        W_output_min(output_counter+(1:q),1) = obj.m_c.boundsOutput.min{k_state}(:,left_or_right);
        output_counter = output_counter + q;
    end
    
    W = [W_output_max;-W_output_min;W_input_max;-W_input_min];
    
    
    
    
    

end
