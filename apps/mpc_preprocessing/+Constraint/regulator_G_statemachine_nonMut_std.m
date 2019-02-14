function  [G]=regulator_G_statemachine_nonMut_std(obj,S_bar)

 eye_size = 0;
 for i=1:obj.state_machine.n_of_models
    dimension = obj.m(i);
    eye_size = eye_size + dimension*sum(obj.state_machine.state_pattern==i);
 end
     
 G    = [S_bar; -S_bar; eye(eye_size); -eye(eye_size)];

end
