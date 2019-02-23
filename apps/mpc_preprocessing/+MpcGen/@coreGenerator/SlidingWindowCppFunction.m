%% it should work even for a single repetition
%% here the idea is that we have three different cases:
%% - one single matrix
%% - one matrix for each step in the prediction windows
%% - and for the last case we can even manage case where we have different version of the matrix for each sample
%%   that can change for each prediction window (it is the case where we have a inizilization sample and then the matrices
%%   get a steady state beahviour)

%% TODO the case with one matrix but an inizialization case (one or more) is not managed yet
function SlidingWindowCppFunction(obj,path_to_folder,all_rep,namefunc,vars,output)
    
    %% TODO check if namefunc and output are char 
    %% and return an error if it does not happen

    % file of c name and header
    funfilename = [char(namefunc),'.cpp'];
    hfilename   = [char(namefunc),'.h'];

    % in all_rep the last columns is always made by all standard matrices
    
    %% i need to get the first non empty element 
    get_out = false;
    for i=1:size(all_rep,1)
        for j =1:size(all_rep,2)
            if(~isempty(all_rep{i,j}))
                first_non_empty_rep = all_rep{i,j};
                get_out = true;
                break;
            end
        end
        if(get_out)
            break;
        end
    end
    
    
    %% with the first ccode initilialize the function structure
    [funstr, hstring] = obj.ccodefunctionstring(first_non_empty_rep,'funname',namefunc,'vars',vars,'output',output);
    
    
    % delimiter for the second split (on the body) to separate the
    % declaration of variables from the actual value assignement
    
    %% creating the .cpp function
    first_split       = strsplit(funstr,{'{','}'});
    signature         = first_split{1};
    body              = first_split{2};
    % I start by working on the signature
    delimiter_for_signature = strcat("double ",output,'[]');
    % i split at the signature of the function
    new_func1           = split(signature,delimiter_for_signature);
    % i split again to get the dimension of the output vector
    new_func2           = split(new_func1{2},[",",")"]);
    new_variable_name   = strcat(output,'_out');
    new_variable_signa  = strcat("double ",new_variable_name);
    new_func3           = split(new_func1{2},["{","}"]);
    new_signature       = "#include ""string.h"" " + newline + new_func1{1} + new_variable_signa +new_func3{1};

    % declare variable inside 
    arr_variable_declare = strcat("double ",output,'[1]',new_func2{1},';');
    % with the second split i separate between varaibles declaration and assignement 
    %matrix_length        = 2*(obj.N*obj.q) + 2*(obj.N*obj.m);
    matrix_length        = length(first_non_empty_rep);
    delimiter            = output + "[0][" + num2str(matrix_length-1) + "]=0;";
    second_split         = strsplit(body,{char(delimiter)});
    var_declaration      = second_split{1};
    %var_assignement      = second_split{2};

    cpp_final_func     = new_signature + "{" + newline + arr_variable_declare + newline+ newline + var_declaration + newline + delimiter +newline; %+ "if(ind1 == 0){" + newline + var_assignement...
                        %+ newline + '}';
    % i start to collect the structure for each variables 
    starting_command_non_standard =  "if(ind_pred_win1 == ";
    inner_blocks_non_standard     =  "else if(ind_pred_win1 == ";
    % here i need to restart and reconsider again the first non null
    % element because we do not know if it is non standard or not
    % here i look if there exist many different version of the matrix (mutable constr, statemachine or both)
    for i = 1:size(all_rep,1)
       % i need to empty both cur_block and all_good_split 
       cur_block = "";
       % in all_good_split the last element is always the standard one
       all_good_split = [];
       for j = 1:(length(obj.non_standard_iteration) + 1)
           if(~isempty(all_rep{i,j}))
               [funstr_cur]      = obj.ccodefunctionstring(all_rep{i,j},'funname',namefunc,'vars',vars,'output',output);
               first_split       = strsplit(funstr_cur,{'{','}'});
               second_split      = strsplit(first_split{2},{char(delimiter)});       
               all_good_split{j} = second_split{2};
           else
               all_good_split{j} = [];
           end
       end
       % here i check if for the current matrices i have different version
       % given the prediction window im considering
       if(length(all_good_split)>1)
           % if i find an occurence of the current problem that is non
           % empty it means that there exist at least one non standard
           % occcurence of the current matrix at the k-th sample time 
           flag_start = false;
           
           for k = 1:(length(all_good_split)-1)
               if(~isempty(all_good_split{k}))
                   if(~flag_start)
                       cur_block = starting_command_non_standard + obj.non_standard_iteration{k}.number + "){"+ newline + all_good_split{1} + newline + "}" + newline;
                       % here i have found at least one occurence of non
                       % standard matrices
                       flag_start = true;
                   else
                       cur_block = cur_block + inner_blocks_non_standard + obj.non_standard_iteration{k}.number + "){" + newline + all_good_split{1} + newline + "}" + newline;
                   end
               end
           end
           % if i enter here it means that there is at least on occurence
           % of non standard matrices in for some samples
           if(flag_start)
               cur_block     = cur_block + "else{" + newline + all_good_split{end} + newline + "}";
           % if i enter here it means that for this case we have only
           % standard matrices and nothing else
           else
               cur_block     = cur_block + newline + all_good_split{end}; %+ newline + "}";
           end
       % it never get into this     
       else
           cur_block  = all_good_split{1};
       end
       % here im looking for the case where i have different occurence of
       % the same matrix over each sample time in the prediction windows
       if(size(all_rep,1)>1)
           if(i == 1)
              cpp_final_func = cpp_final_func + "if(ind1 == 0)" +"{" + newline + cur_block + newline + "}";
           else
               cpp_final_func = cpp_final_func + "else if(ind1 == "+ num2str(i-1) + "){" + newline + cur_block + newline + "}";
           end
       else
           % here we do not have any multiple definition of the same matrix
           % of the optimization problem (neither mutable nor state_machine)
           cpp_final_func = cpp_final_func + "{" + newline + cur_block + newline + "}";
       end
    end

    % adding last memcopy 
    vector_dimension    = erase(new_func2{1},["[","]"]);
    copy_to_out         = strcat("memcpy(",new_variable_name,",",output,"[0]",",sizeof(double)*",vector_dimension,");");
    cpp_final_func      = cpp_final_func + newline + copy_to_out;
    % adding closing bracket
    cpp_final_func      = cpp_final_func + newline + "}";

    %% fixing the .h function
    delimiter_for_split = strcat("double ",output,'[]',new_func2{1});
    new_variable_signa  = strcat("double ",new_variable_name,new_func2{1});
    new_func1           = split(hstring,delimiter_for_split);
    % change the signature of the function in the header file
    % important!!! in this function it is expecting a function
    % name inside the comment at the beggining of the header too
    new_hstring         = new_func1{1} + new_variable_signa + new_func1{2};
    
   
    
    %% Generate C implementation file
    fid = fopen(fullfile(convertStringsToChars(obj.basepath),funfilename),'w+');

    % Includes
    fprintf(fid,'%s\n\n',...
        ['#include "',path_to_folder,'/',hfilename,'"']);

    % Function
    fprintf(fid,'%s\n\n',cpp_final_func);
    fclose(fid);

    %% Generate C header file
    
    % Create the function description header
    hStruct = obj.createHeaderStruct(namefunc); % create header
    hStruct.calls = new_hstring;
    hFString = obj.cHeader(hStruct);
    
    fid = fopen(fullfile(convertStringsToChars(obj.basepath),hfilename),'w+');

    % Header
    fprintf(fid,'%s\n\n',hFString);

    % Include guard
    fprintf(fid,'%s\n%s\n\n',...
        ['#ifndef ', upper([namefunc,'_h'])],...
        ['#define ', upper([namefunc,'_h'])]);

    % Includes
    fprintf(fid,'%s\n\n',...
        '#include "math.h"');

    % Function prototype
    fprintf(fid,'%s\n\n',new_hstring);

    % Include guard
    fprintf(fid,'%s\n',...
        ['#endif /*', upper([namefunc,'_h */'])]);

    fclose(fid);
end