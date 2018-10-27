classdef coreGenerator <  handle
    
    properties 
        % general information about path and type of solver
        basepath    %
        type
        solver
        % structure of the problem
        n           % state space dim
        m           % control space dim 
        q           % output space dim 
        delta       % sampling time
        N           % widht of prediction window
        N_constr    % number of constraints
        
        %  for code generation
        sym_H       %             
        sym_F_tra   %
        sym_G       %
        sym_W       %
        sym_S       %
        x_0         %
        % for control
        H
        F_tra
        G
        W
        S
       
        
    end
    
%     methods(Abstract) 
%        
%     end
   
    methods
       
        function obj = coreGenerator(type,solver,generate_functions)
            
            obj.type   = type;
            obj.solver = solver;
            obj.GetBasePath();
            if(generate_functions)
                if ~exist(obj.basepath,'dir')
                    mkdir(convertStringsToChars(obj.basepath));
                end
            end
            
            
        end
       
       
       function GetBasePath(obj)
          
           % get time stamp
           dt     = datestr(now,'mmmm dd, yyyy HH:MM:SS.FFF AM');
           % folder name
           folder = strcat(obj.solver,'_',obj.type,'_',dt);
           % find location of the current folder
           pth = which('mainFunctionGen.m');
           % take only the path till the container folder of current file
           pth = fileparts(pth);
           obj.basepath = strcat(pth,'/generated_function/',folder);
       end
       
       
       function GenFunctions(obj) 
            if(strcmp(obj.type,"fixed") && strcmp(obj.solver,"QPoases"))
               
               %% optmization problem formulation for QPoases
               % J(z) = z'*H*z + g*z
               % s.t
               %    l_b <= z <= u_b
               %    l_b <= A*z <= u_b
               
               %% hessian cost function
               H_  = obj.sym_H(:);
               obj.cCode(H_,'compute_H',{},'H');
               obj.PostProcessFunctionForqpOASES('compute_H','H')
               %% linear term cost function
               g_  = (obj.x_0'*obj.sym_F_tra)'; 
               obj.cCode(g_,'compute_g',{obj.x_0},'g');
               obj.PostProcessFunctionForqpOASES('compute_g','g')
               %% linear term constraints
               A_  = obj.sym_G(:);
               obj.cCode(A_,'compute_A',{},'A');
               obj.PostProcessFunctionForqpOASES('compute_A','A')
               %% constant term constraints
               ub_ = obj.sym_W + obj.sym_S*obj.x_0;
               obj.cCode(ub_,'compute_ub',{obj.x_0},'ub');
               obj.PostProcessFunctionForqpOASES('compute_ub','ub')
            end         
       end
       
       function PostProcessFunctionForqpOASES(obj,filename,namefunc)
               %% working on .c file
               filepath            = strcat(obj.basepath,'/',filename,'.c');
               str                 = fileread(char(filepath));
               delimiter_for_split = strcat("double ",namefunc,'[]');
               % i split at the signature of the function
               new_func1           = split(str,delimiter_for_split);
               % i split again to get the dimension of the output vector
               new_func2           = split(new_func1{2},[",",")"]);
               % here i create all the pieces fo the new function that im
               % going to stitch togheter
               new_variable_name   = strcat(namefunc,'_out');
               new_variable_signa  = strcat("double ",new_variable_name);
               variable_declare    = strcat("double ",namefunc,'[1]',new_func2{1});
               vector_dimension    = erase(new_func2{1},["[","]"]);
               copy_to_out         = strcat("memcpy(",new_variable_name,",",namefunc,"[0]",",sizeof(double)*",vector_dimension,");");
               % last split 
               new_func3           = split(new_func1{2},["{","}"]);
               
               % reconstruct new func for .cpp
               new_string = "#include ""string.h"" " + newline + new_func1{1} + new_variable_signa +new_func3{1}+ "{" + newline + variable_declare + ";" + newline + new_func3{2} ...
                            + newline + copy_to_out  + newline + "}";
               
               % save the new func as .cpp
               new_name_file = strcat(obj.basepath,'/',filename,'.cpp');
               fid = fopen(new_name_file,'w');
               fprintf(fid,'%s',new_string);
               fclose(fid);
               
               % memcpy(H_out,H[0], sizeof(double)*64);
               
               % delete old .c file
               delete(char(filepath));
               
               %% working on .h file
               filepath            = strcat(obj.basepath,'/',filename,'.h');
               str                 = fileread(char(filepath));
               delimiter_for_split = strcat("double ",namefunc,'[]',new_func2{1});
               new_variable_signa  = strcat("double ",new_variable_name,new_func2{1});
               new_func1           = split(str,delimiter_for_split);
               % change the signature of the function in the header file
               % important!!! in this function it is expecting a function
               % name inside the comment at the beggining of the header too
               new_string = new_func1{1} + new_variable_signa + new_func1{2} + new_variable_signa + new_func1{3};
               % save the new func as .h
               fid = fopen(filepath,'w');
               fprintf(fid,'%s',new_string);
               fclose(fid);              
       end
       
       function GenParametersFile(obj)
           
           filepath  = strcat(obj.basepath,'/parameters.xml');
           
           pNode     = com.mathworks.xml.XMLUtils.createDocument('parameters');
           
           entry_node = pNode.createElement('Entry');
           pNode.getDocumentElement.appendChild(entry_node);
         
           name_node = pNode.createElement('n');
           name_text = pNode.createTextNode(int2str(obj.n));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('m');
           name_text = pNode.createTextNode(int2str(obj.m));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('q');
           name_text = pNode.createTextNode(int2str(obj.q));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           %name_node = pNode.createElement('delta');
           %name_text = pNode.createTextNode(int2str(obj.delta));
           %name_node.appendChild(name_text);
           %entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('N');
           name_text = pNode.createTextNode(int2str(obj.N));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('N_constr');
           name_text = pNode.createTextNode(int2str(obj.N_constr));
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('type');
           name_text = pNode.createTextNode(obj.type);
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           name_node = pNode.createElement('solver');
           name_text = pNode.createTextNode(obj.solver);
           name_node.appendChild(name_text);
           entry_node.appendChild(name_node);
           
           xmlwrite(char(filepath),pNode);   
       end
       
        
       
       
    end
    
    
end