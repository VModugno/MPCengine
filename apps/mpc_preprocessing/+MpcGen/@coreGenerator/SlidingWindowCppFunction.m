function SlidingWindowCppFunction(obj,path_to_folder,all_rep,namefunc,vars,output)
%% with the first ccode initilialize the function structure
    [funstr, hstring] = obj.ccodefunctionstring(all_rep{1},'funname',char(namefunc),'vars',vars,'output',char(output));
    
    % file of c name and header
    funfilename = [char(namefunc),'.cpp'];
    hfilename   = [char(namefunc),'.h'];
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
    matrix_length        = length(all_rep{1});
    delimiter            = output + "[0][" + num2str(matrix_length-1) + "]=0;";
    second_split         = strsplit(body,{char(delimiter)});
    var_declaration      = second_split{1};
    var_assignement      = second_split{2};

    cpp_final_func     = new_signature + "{" + newline + arr_variable_declare + newline+ newline + var_declaration + newline + delimiter +newline + "if(ind1 == 0){" + newline + var_assignement...
                        + newline + '}';
    % i start to collect the structure for each variables 
    for i = 2:obj.N
       [funstr_cur]   = obj.ccodefunctionstring(all_rep{i},'funname',char(namefunc),'vars',vars,'output',char(output));
       first_split    = strsplit(funstr_cur,{'{','}'});
       second_split   = strsplit(first_split{2},{char(delimiter)});       
       cpp_final_func = cpp_final_func + "else if(ind1=="+ num2str(i-1) + "){" + newline + second_split{2} + newline + "}";

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