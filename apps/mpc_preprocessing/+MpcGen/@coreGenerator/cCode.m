
function cCode(obj,sym,fun_name,variables,output)
    
    % file of c name and header
    funfilename = [fun_name,'.c'];
    hfilename   = [fun_name,'.h'];
    
    % Convert symbolic expression into C-code
    [funstr, hstring] = obj.ccodefunctionstring(sym,'funname',fun_name,'vars',variables,'output',output);

    % Create the function description header
    hStruct = obj.createHeaderStruct(fun_name); % create header
    hStruct.calls = hstring;
    hFString = obj.cHeader(hStruct);

    %% Generate C implementation file
    fid = fopen(fullfile(convertStringsToChars(obj.basepath),funfilename),'w+');

    % Includes
    fprintf(fid,'%s\n\n',...
        ['#include "', hfilename,'"']);

    % Function
    fprintf(fid,'%s\n\n',funstr);
    fclose(fid);

    %% Generate C header file
    fid = fopen(fullfile(convertStringsToChars(obj.basepath),hfilename),'w+');

    % Header
    fprintf(fid,'%s\n\n',hFString);

    % Include guard
    fprintf(fid,'%s\n%s\n\n',...
        ['#ifndef ', upper([fun_name,'_h'])],...
        ['#define ', upper([fun_name,'_h'])]);

    % Includes
    fprintf(fid,'%s\n\n',...
        '#include "math.h"');

    % Function prototype
    fprintf(fid,'%s\n\n',hstring);

    % Include guard
    fprintf(fid,'%s\n',...
        ['#endif /*', upper([fun_name,'_h */'])]);

    fclose(fid);
end