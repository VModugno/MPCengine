
%% here the hypothesis is that the footstep duration is exactly N/2 so we have only three footstep in the prediction horizon of our MPC

function  [G]=regulator_G_statemachine_pattern_walkingRotation(obj,S_bar)


    all_R_for_input_constraints = cell(1);
    % for now the choice of foot switching is fixed by the choice of N/2 
    % to alter the foot generation procedure (number of steps) it will
    % require a deep reworking of all these functions
    inner_index = 1;
    for kk = 1:(obj.N/2):obj.N
        cur_theta = obj.inner_x_ext(kk);
        % 
        % this matrix already contains R transpose
        cur_R_t = [1,       0,       0, 0;
                 0 cos(cur_theta), 0    sin(cur_theta);
                 0        0        1    0;             
                 0 -sin(cur_theta) 0    cos(cur_theta)];


        % i store the resulting value inside all A and all B
        all_R_for_input_constraints{inner_index,1} = cur_R_t;
        inner_index = inner_index + 1;
    end

    % here i build the dimension of the matrices
    eye_size = 0;
    for i=1:obj.state_machine.n_of_models
        dimension = obj.m(i);
        eye_size = eye_size + dimension*sum(obj.state_machine.state_pattern==i);
    end
    
    G_component = sym(eye(eye_size));
    
    % here i position the rotation matrices in the right location
    c_c = 0;
    inner_counter = 1;
    for jj = 1:length(obj.state_machine.state_pattern)
        
        %here i assume that the state model where i have to attach the
        %rotation is equal to one, NON GENERAL!
        if(obj.state_machine.state_pattern(jj) == 1)
            G_component((c_c + 1):(c_c + 4),(c_c + 1):(c_c + 4)) = all_R_for_input_constraints{inner_counter};
            inner_counter = inner_counter + 1;
        end
         % update current_coordinates aka c_c
         c_c = c_c + obj.m(obj.state_machine.state_pattern(jj));
    end
    
    % DEBUG
    %S_bar_2_c = double(subs(S_bar,obj.inner_x_ext,[ 0 0 0 0 0 0]'));
    

    G    = [S_bar; -S_bar; G_component; -G_component];

end
