function  [S]=tracker_S_nonMut_std(obj,T_bar_C)
S     = [-T_bar_C; T_bar_C; zeros(obj.N*obj.m,obj.n); zeros(obj.N*obj.m,obj.n)];
end