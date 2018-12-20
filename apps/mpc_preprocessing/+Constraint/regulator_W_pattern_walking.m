function W = regulator_W_pattern_walking(obj)
    part_W_max = zeros(obj.N*obj.q,1);
    part_W_min = zeros(obj.N*obj.q,1);
    for jj = 1:obj.m_c.N_state
        part_W_max  = part_W_max + kron(obj.m_c.const_pattern(:,jj), obj.m_c.boundsOutput.max(:,jj));
        if( isfield(obj.m_c.boundsOutput,'min') && ~isempty(obj.m_c.boundsOutput.min))
            part_W_min  = part_W_min + kron(obj.m_c.const_pattern(:,jj), -obj.m_c.boundsOutput.min(:,jj));
        end
    end
    if( isfield(obj.m_c.boundsOutput,'min') && ~isempty(obj.m_c.boundsOutput.min))
        W = [part_W_max;part_W_min];
    else
        W = [part_W_max;part_W_max];
    end


    % adding constraints about input
    W =[W;
       kron(ones(obj.N,1), obj.m_c.boundsInput.max);
       kron(ones(obj.N,1), obj.m_c.boundsInput.max)]; 
end
