% this function substitute cCode for the statemachine (only regulator for now)
% 
function StateMachineObjective_QPOASES(obj,input,namefunc,path_to_folder,vars,output)
   
    all_rep   = cell(2,1);
    all_rep_H = cell(1,obj.N);
    all_rep_g = cell(1,obj.N);
    obj.state_machine.reset();
    if(strcmp(obj.problemClass,"regulator"))
        for i=1:obj.N 
            S_bar_obj   = input.S_bar_obj{i};
            T_bar_obj   = input.T_bar_obj{i};
            Q_bar       = input.Q_bar{i};
            R_bar       = input.R_bar{i};
            [H_,F_tra_] = eval(obj.costFuncCall);
            H_          = H_';
            all_rep_H{i}= vpa(H_(:));
            all_rep_g{i}= vpa((obj.x_0'*F_tra_)');
            obj.UpdateStateMachinePattern();
        end   
    elseif(strcmp(obj.problemClass,"tracker"))
        %% here i can recall ref_0 and x_0 and u_prev from object
    end
    
    all_rep{1} = all_rep_H;
    all_rep{2} = all_rep_g;
    
    for i =1:length(namefunc)
        obj.SlidingWindowCppFunction(path_to_folder,all_rep{i},char(namefunc(i)),vars,char(output(i)));
    end


end