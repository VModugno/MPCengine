function  [W]=regulator_W_std(obj)

 W     = [kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxOutput); kron(ones(obj.N,1),obj.maxInput); kron(ones(obj.N,1),obj.maxInput)];

end