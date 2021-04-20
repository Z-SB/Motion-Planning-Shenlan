function hn = heuristic(x1,y1,x2,y2,option)
%This function calculates heuristic cost according to the option
%
if(option==0)  %Dijstla
    hn = 0;
elseif(option==1)  % A* using Eucliean
    hn=sqrt((x1-x2)^2 + (y1-y2)^2);
    
elseif(option==2)   % A* using Mahatten solution
    dx = abs(x2-x1);
    dy = abs(y2-y1);
    hn=dx+dy;
    
elseif(option==3)   % A* using optimal solution
    dx = abs(x2-x1);
    dy = abs(y2-y1);
    hn=dx+dy+(sqrt(2)-2)*min(dx,dy);
    
end
            
        
    
%   Copyright 2009-2010 The MathWorks, Inc.

