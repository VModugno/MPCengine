% this function substitute cCode for the mutable constraints case 
%% TODO in general make a distinciton between functions and mutable does not make a lot fo sense
function StateMachineConstraints_QPOASES(obj,input,namefunc,path_to_folder,vars,output)
                                     
    all_rep_A  = cell(obj.N,length(obj.non_standard_iteration)+1);
    all_rep_ub = cell(obj.N,length(obj.non_standard_iteration)+1);
    if(strcmp(obj.problemClass,"regulator"))
        obj.ResetStateConstrPattern();
        obj.ResetStateMachinePattern();
        
        non_standard_iter = 1;
        % here we make the hypothesis that the non standard sequence are
        % consecutive if there exist more than one
        if(~isempty(obj.non_standard_iteration))
            non_standard_iter = length(obj.non_standard_iteration);
        end
        
        % we do +1 on the interation bound of the extnernal for because we need to processe all the non standard
        % iter and than we go for the standard one
        for j=1:non_standard_iter + 1
            for i=1:obj.N 
               S_bar_constr = input.S_bar_constr{i};
               T_bar_constr = input.T_bar_constr{i}; 
               %% building A
               if(strcmp(obj.m_c.g,"pattern")) 
                    A_           = obj.MutableConstraints_G(obj,S_bar_constr)';
               else
                    A_           = eval(obj.constrFuncG_Call)'; 
               end
               if(j<non_standard_iter + 1)
                   if(obj.non_standard_iteration_flag)
                       all_rep_A{i,j} = vpa(A_(:));
                       obj.non_standard_iteration_flag = false;
                   else
                       all_rep_A{i,j} = [];
                   end
               else
                   all_rep_A{i,j} = vpa(A_(:));
               end
               %% building ub
               if(strcmp(obj.m_c.w,"pattern")) 
                    W = obj.MutableConstraints_W(obj);
               else
                    W = eval(obj.constrFuncW_Call);
               end

               if(strcmp(obj.m_c.s,"pattern")) 
                    S = obj.MutableConstraints_S(obj,T_bar_constr);
               else
                    S = eval(obj.constrFuncS_Call);
               end
               % here i wrote all the non standard matrices
               % only when they appear
               if(j<non_standard_iter + 1)
                   if(obj.non_standard_iteration_flag)
                       all_rep_ub{i,j} = vpa(W + S*obj.x_0);
                       obj.non_standard_iteration_flag = false;
                   else
                       all_rep_ub{i,j} = [];
                   end
               % here i wrote the last one standard 
               else
                   all_rep_ub{i,j} = vpa(W + S*obj.x_0);
               end

               obj.UpdateStateMachinePattern();
               obj.UpdateConstrPattern();
            end  
        end
    elseif(strcmp(obj.problemClass,"tracker"))
%         if(strcmp(output,"A"))
%             for i=1:obj.N 
%                 A_        = obj.MutableConstraints_G(obj,obj.m_c.S_bar_C)';
%                all_rep{i} = vpa(A_(:));
%                obj.UpdateConstrPattern();
%             end   
%         elseif(strcmp(output,"ub"))
%             % i compute all the posible W combination inside the prediction window
%             all_W = zeros(2*(obj.N*obj.q) + 2*(obj.N*obj.m),obj.N);
%             all_S = zeros(2*(obj.N*obj.q) + 2*(obj.N*obj.m),obj.N*obj.n);
% 
%             jj = 1;
%             for i=1:obj.N 
%                if(strcmp(obj.m_c.w,"pattern")) 
%                     all_W(:,i) = obj.MutableConstraints_W(obj,obj.u_cur);
%                else
%                     all_W(:,i) = obj.sym_W;
%                end
% 
%                if(strcmp(obj.m_c.s,"pattern")) 
%                     all_S(:,jj:jj+(obj.n - 1) ) = obj.MutableConstraints_S(obj,obj.m_c.T_bar_C);
%                else
%                     all_S(:,jj:jj+(obj.n- 1) ) = obj.sym_S;
%                end
%                jj = jj + obj.n;
%                obj.UpdateConstrPattern();
%             end
%             % i compute all the function ub_ to stitch togheter 
%             jj = 1;
%             for i=1:obj.N
%                all_rep{i} = vpa(all_W(:,i) + all_S(:,jj:jj+(obj.n-1) )*var_for_ub);
%                jj = jj + obj.n;
%             end
%         end  
    end
    
    all_rep{1} = all_rep_A;
    all_rep{2} = all_rep_ub;
    
    for i =1:length(namefunc)
        obj.SlidingWindowCppFunction(path_to_folder,all_rep{i},char(namefunc(i)),vars,char(output(i)));
    end
    
end