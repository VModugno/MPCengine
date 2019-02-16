%% using vpa over the symbolic expression prevent rounding issue on cpp when it comes to evaluate symbolic numerical division for example

function QPOASESgenFunc(obj)
   %% optmization problem formulation for QPoases
   %% inportant for QPoases matrix has to be stored row wise
   % J(z) = z'*H*z + g*z
   % s.t
   %    l_b <= z <= u_b
   %    l_b <= A*z <= u_b
   %% here i define the structure of the inner_x
   if(strcmp(obj.problemClass,"tracker"))
        %inner_x = [obj.x_0;obj.u_0;obj.ref_0;obj.index];
         inner_x = [obj.u_prev;obj.x_0;obj.inner_x_ext;obj.ref_0;obj.index];
   else
       % if the system is not LTV obj.inner_x_ext is gonna be
       % zero
        inner_x = [obj.x_0;obj.inner_x_ext;obj.index];
   end
   %% hessian cost function 
   if(~isempty(obj.state_machine))
       out = obj.StateMachineGenerator();
       disp('generating H,g')
       StateMachineObjective_QPOASES(out,ref_0,x_0,u_prev,['compute_H','compute_g'],'current_func',{inner_x,obj.outer_x},['H','g'])
   else
       disp('generating H')
       %H_  = obj.sym_H(:);
       H_  = obj.sym_H';
       H_  = vpa(H_(:));
       obj.cCode(H_,'compute_H','current_func',{inner_x,obj.outer_x},'H');
       obj.PostProcessFunctionForqpOASES('compute_H','H')
       %% linear term cost function
       disp('generating g')
       if(strcmp(obj.problemClass,"tracker"))
            g_  = vpa(([obj.ref_0;obj.x_0;obj.u_prev]'*obj.sym_F_tra)'); 
       else
            g_  = vpa((obj.x_0'*obj.sym_F_tra)');
       end
       obj.cCode(g_,'compute_g','current_func',{inner_x,obj.outer_x},'g');
       obj.PostProcessFunctionForqpOASES('compute_g','g')
       %% linear term constraints
       disp('generating A')
       if(strcmp(obj.m_c.g,"pattern"))
           obj.MutableConstraints_QPOASES([],'compute_A','current_func',{inner_x,obj.outer_x},'A');
       else
           A_  = obj.sym_G';
           A_  = vpa(A_(:)); % we need vpa to avoid rounding problem on cpp
           obj.cCode(A_,'compute_A','current_func',{inner_x,obj.outer_x},'A');
           obj.PostProcessFunctionForqpOASES('compute_A','A')
       end
       %% constant term constraints
       disp('generating ub')
       if(strcmp(obj.problemClass,"tracker"))
           if(strcmp(obj.m_c.w,"pattern") || strcmp(obj.m_c.s,"pattern"))
                % with this functions i write the function and i correct it 
                obj.MutableConstraints_QPOASES([obj.x_0;obj.u_prev],'compute_ub','current_func',{inner_x,obj.outer_x},'ub');
           else
                ub_ = vpa(obj.sym_W + obj.sym_S*[obj.x_0;obj.u_prev]);
                % with this functions i write the function and i correct it 
                obj.cCode(ub_,'compute_ub','current_func',{inner_x,obj.outer_x},'ub');
                obj.PostProcessFunctionForqpOASES('compute_ub','ub') 
           end
       elseif(strcmp(obj.problemClass,"regulator"))
            if(strcmp(obj.m_c.w,"pattern") || strcmp(obj.m_c.s,"pattern"))
                % with this functions i write the function and i correct it 
                obj.MutableConstraints_QPOASES(obj.x_0,'compute_ub','current_func',{inner_x,obj.outer_x},'ub');
            else
                ub_ = vpa(obj.sym_W + obj.sym_S*obj.x_0);
                % with this functions i write the function and i correct it 
                obj.cCode(ub_,'compute_ub','current_func',{inner_x,obj.outer_x},'ub');
                obj.PostProcessFunctionForqpOASES('compute_ub','ub') 
            end
       end
   end


end