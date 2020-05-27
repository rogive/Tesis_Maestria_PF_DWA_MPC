function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************

%start = [50, 350];
%goal = [400, 50];

current = start_coords;
route = start_coords;
next_coords = [];

% 1. On every iteration the planner should update the position of the robot based on the gradient
%values contained in the arrays gx and gy. Make sure you normalize the gradient vectors.

for i = 1:max_its
    yidx = current(2); 
    xidx = current(1);
    
    % gx size (400 rows 600 cols)
    % gy size (400 rows 600 cols)
    % current 50 cols 350 rows
    V = [gx(yidx, xidx) gy(yidx, xidx)];
    ngV = V/norm(V);

% 2. Update the route by adding the new position of the robot to the end of the route array. Note
%that the distance between successive locations in the route should not be greater than 1.0.

    % velocity follows nomalized gx gy
    % next_coords = [round(current(1)+ngV(1)) round(current(2)+ngV(2))];

    % uniform velocity 1/step
    if (ngV(1) > 0) && (ngV(2) > 0)
        if abs(ngV(1)) > abs(ngV(2))
            next_coords = [current(1)+1 current(2)];
        else
            next_coords = [current(1) current(2)+1];
        end
    end
    
    if (ngV(1) < 0) && (ngV(2) > 0)
        if abs(ngV(1)) > abs(ngV(2))
            next_coords = [current(1)-1 current(2)];
        else
            next_coords = [current(1) current(2)+1];
        end
    end

    
    if (ngV(1) < 0) && (ngV(2) < 0)
        if abs(ngV(1)) > abs(ngV(2))
            next_coords = [current(1)-1 current(2)];
        else
            next_coords = [current(1) current(2)-1];
        end
    end
    
    if (ngV(1) > 0) && (ngV(2) < 0)
        if abs(ngV(1)) > abs(ngV(2))
            next_coords = [current(1)+1 current(2)];
        else
            next_coords = [current(1) current(2)-1];
        end
    end
    
    current = next_coords;    
    route = [route; current];    
    next_coords;
    route;
    
% 3. Continue the same procedure until the distance between the robot’s current position and the 
%goal is less than 2.0 or the number of iterations exceeds the value contained in max its.
  
    if pdist([current; end_coords],'euclidean') < 2
        disp('end');
        break;
    end
    
end

% *******************************************************************
end