function  [W]=tracker_W_nonMut_std(obj,u_prev)

    if (isfield(obj.B_Out,'min')  && ~isempty(obj.B_Out.min) )
        if( isfield(obj.B_In,'min')  && ~isempty(obj.B_In.min) ) % out min true in min true
            W     = [kron(ones(obj.N,1),obj.B_Out.max); kron(ones(obj.N,1),-obj.B_Out.min);...
                    -kron(ones(obj.N,1),u_prev)+kron(ones(obj.N,1),obj.B_in.max); kron(ones(obj.N,1),u_prev) + kron(ones(obj.N,1),-obj.B_in.min)];
        else % out min true in min false
            W     = [kron(ones(obj.N,1),obj.B_Out.max); kron(ones(obj.N,1),-obj.B_Out.min);...
                    -kron(ones(obj.N,1),u_prev)+kron(ones(obj.N,1),obj.B_in.max); kron(ones(obj.N,1),u_prev) + kron(ones(obj.N,1),obj.B_in.max)];
        end
    elseif( isfield(obj.B_In,'min')  && ~isempty(obj.B_In.min) ) % out min false in min true
        W     = [kron(ones(obj.N,1),obj.B_Out.max); kron(ones(obj.N,1),obj.B_Out.max);...
                -kron(ones(obj.N,1),u_prev)+kron(ones(obj.N,1),obj.B_In.max); kron(ones(obj.N,1),u_prev) + kron(ones(obj.N,1),-obj.B_In.min)]; 
    else % out min false in min false
        W     = [kron(ones(obj.N,1),obj.B_Out.max); kron(ones(obj.N,1),obj.B_Out.max);...
                -kron(ones(obj.N,1),u_prev)+kron(ones(obj.N,1),obj.B_In.max); kron(ones(obj.N,1),u_prev) + kron(ones(obj.N,1),obj.B_In.max)];

    end
 
end