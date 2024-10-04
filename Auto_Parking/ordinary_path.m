function [x,y,theta] = ordinary_path(PX,PY)
%0819:修复了只有两段情况下的theta值无限的问题
    if PX(1) ~= PX(2)
        x = PX(1):(PX(2)-PX(1))/499:PX(2); 
    else
        x = ones(1,500) * PX(1);
    end
    
    if PY(1) ~= PY(2)
        y = PY(1):(PY(2)-PY(1))/499:PY(2);
    else
        y = ones(1,500) * PY(1);
    end
    
    if PX(1) == PX(2) && PY(1) == PY(2)
        theta = ones(1,500) * qtan(PY(1),PX(1),PY(3),PX(3));
    else
        theta = ones(1,500) * qtan(PY(1),PX(1),PY(2),PX(2));
    end
end

function theta=qtan(y1,x1,y2,x2)
    
theta=atan((y2-y1)/(x2-x1));
%First Quadrant
if (x2>x1)&&(y2>y1)
    theta =abs(theta);
    
%90 degrees
elseif (x2==x1)&&(y2>y1)
    theta = pi/2;
   
%180 degrees
elseif (x2<x1)&&(y2==y1)
    theta = pi;
    
%270 degrees
elseif (x2==x1)&&(y2<y1)
    theta =-pi/2;   
    
%Second Quadrant
elseif (x2<x1)&&(y2>y1)
    theta = pi - abs(theta);
    
%Third Quadrant
elseif (x2<x1)&&(y2<y1)
    theta = abs(theta) -pi;
    
%Fourth Quadrant
elseif (x2>x1)&&(y2<y1)
    theta = -abs(theta);
    
%Zero
elseif (x2>x1)&&(y2==y1)
    theta =0;
    
end

end