% this function substitute cCode for the statemachine 
% 
function StateMachineObjective_QPOASES(obj,input,ref_0,x_0,u_prev,namefunc,path_to_folder,vars,output)
    % file of c name and header
    for i =1:length(namefunc)
        funfilename = [namefunc(i),'.cpp'];
        hfilename   = [namefunc(i),'.h'];
    end

    all_rep = cell(obj.N,2);
  
    if(strcmp(obj.problemClass,"regulator"))
        for i=1:obj.N 
            S_bar_obj   = input.S_bar_obj{i};
            T_bar_obj   = input.T_bar_obj{i};
            Q_bar       = input.Q_bar{i};
            R_bar       = input.R_bar{i};
            [H_,F_tra_] = eval(obj.costFuncCall);
            H_          = H_';
            all_rep{i,1}= vpa(H_(:));
            all_rep{i,2}= vpa((x_0'*F_tra_)');
            obj.UpdateStateMachinePattern();
        end   
    elseif(strcmp(obj.problemClass,"tracker"))
        
    end
    
    for i =1:length(namefunc)
        obj.SlidingWindowCppFunction(path_to_folder,all_rep{i},namefunc,vars,output(i));
    end


end