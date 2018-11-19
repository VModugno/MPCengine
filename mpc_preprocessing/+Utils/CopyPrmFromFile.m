function new_prm = CopyPrmFromFile(conf,cur_prm)

    conf_file = "ConfigFile."+ conf;
    eval(conf_file);

    
    param_fields = fieldnames(cur_prm);
    
    for i = 1:numel(param_fields)
        found = false;
        for j = 1:length(param_list)
            if(strcmp(param_fields{i},param_list(j)))
                new_prm.(param_fields{i}) = value_(1,j);
                found = true;
                break;
            end 
        end
        if(~found)
            new_prm.(param_fields{i}) = cur_prm.(param_fields{i});
        end
    end
    
    
end