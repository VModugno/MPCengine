function  [G]=tracker_G_nonMut_std(obj,S_bar_C)
    G     = [ S_bar_C;-S_bar_C; tril(ones(obj.N*obj.m)); -tril(ones(obj.N*obj.m))];
end