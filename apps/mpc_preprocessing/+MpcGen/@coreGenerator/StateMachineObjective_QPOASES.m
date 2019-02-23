% this function substitute cCode for the statemachine (only regulator for now)
% 
function StateMachineObjective_QPOASES(obj,input,namefunc,path_to_folder,vars,output)
   
    all_rep   = cell(2,1);
    all_rep_H = cell(obj.N,length(obj.non_standard_iteration) + 1);
    all_rep_g = cell(obj.N,length(obj.non_standard_iteration) + 1);
    obj.ResetStateMachinePattern();
    
    non_standard_iter = 1;
    % here we make the hypothesis that the non standard sequence are
    % consecutive if there exist more than one
    if(~isempty(obj.non_standard_iteration))
        non_standard_iter = length(obj.non_standard_iteration);
    end
    
    
    if(strcmp(obj.problemClass,"regulator"))
        for j=1:non_standard_iter + 1
            for i=1:obj.N 
                S_bar_obj   = input.S_bar_obj{i};
                T_bar_obj   = input.T_bar_obj{i};
                Q_bar       = input.Q_bar{i};
                R_bar       = input.R_bar{i};
                [H_,F_tra_] = eval(obj.costFuncCall);
                H_          = H_';
                if(j<non_standard_iter + 1)
                    if(obj.non_standard_iteration_flag)    
                        all_rep_H{i,j}= vpa(H_(:));
                        all_rep_g{i,j}= vpa((obj.x_0'*F_tra_)');
                        obj.non_standard_iteration_flag = false;
                    else
                        all_rep_A{i,j} = [];
                    end
                else
                    all_rep_H{i,j}= vpa(H_(:));
                    all_rep_g{i,j}= vpa((obj.x_0'*F_tra_)');
                end
                obj.UpdateStateMachinePattern();
            end
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