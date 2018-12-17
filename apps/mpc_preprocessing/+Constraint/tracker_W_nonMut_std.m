function  [W]=tracker_W_nonMut_std(obj,u_prev)
 W     = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput);...
         -kron(ones(obj.N,1),u_prev)+kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),u_prev) + kron(ones(obj.N,1),obj.maxInput)];
end