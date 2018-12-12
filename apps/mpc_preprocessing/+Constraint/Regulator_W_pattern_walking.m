function W = Regulator_W_pattern_walking(obj)
    part_W = zeros(obj.N*obj.q,1);
    for jj = 1:obj.m_c.N_state
        part_W  = part_W + kron(obj.m_c.const_pattern(:,jj), obj.m_c.bounds(:,jj));
    end
    W = [part_W;part_W];


    % adding constraints about input
    W =[W;
       kron(ones(obj.N,1), obj.maxInput);
       kron(ones(obj.N,1), obj.maxInput)]; 
end