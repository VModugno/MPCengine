function  [S]=regulator_S_statemachine_nonMut_std(obj,T_bar)

 rows = 0;
 for i=1:obj.state_machine.n_of_models
    dimension = obj.m(i);
    rows = rows + dimension*sum(obj.state_machine.state_pattern==i);
 end

 S  = [-T_bar; T_bar; zeros(rows,obj.n(1)); zeros(rows,obj.n(1))];

end
