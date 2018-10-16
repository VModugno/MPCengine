classdef (Abstract) AbstractController <  handle
    
    
   methods(Abstract) 
       tau=ComputeControl(obj,x_cur,varargin);
       PlotGraph(obj);
   end
   
    
    
end