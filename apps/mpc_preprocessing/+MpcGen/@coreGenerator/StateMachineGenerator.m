% this function creates the set of all possible matrices (one for each time sample in the prediction windows)
% that can be used for computing the 
function [out]= StateMachineGenerator(obj)
    
    A        = obj.A;
    B        = obj.B;
    C_constr = obj.C_constr;
    C_obj    = obj.C_obj;
    Q        = obj.Q;
    R        = obj.R;
    
    
    for i=1:obj.N 
        [S_bar_obj,S_bar_constr,T_bar_obj,T_bar_constr,Q_bar,R_bar] = eval(obj.propModelCall);
        out.S_bar_obj{i}    = S_bar_obj;
        out.S_bar_constr{i} = S_bar_constr;
        out.T_bar_obj{i}    = T_bar_obj;
        out.T_bar_constr{i} = T_bar_constr;
        out.Q_bar{i}        = Q_bar;
        out.R_bar{i}        = R_bar;
        obj.UpdateStateMachinePattern();
    end

    
end