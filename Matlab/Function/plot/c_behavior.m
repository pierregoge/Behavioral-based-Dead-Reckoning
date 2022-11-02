function [box_color] = c_behavior(ethogram,v)

if v == 1
    if ethogram(1,1) == 1 % SWIM
        box_color(1,1:3) =  [0 0 1];
    end
    if ethogram(1,1) == 2 % REST
        box_color(1,1:3) =  [0 0 0];
    end
    if ethogram(1,1) == 3 % UP
        box_color(1,1:3) =  [0 1 0];
    end
    if ethogram(1,1) == 4 % DOWN
        box_color(1,1:3) =  [1 0 0];
    end
    if ethogram(1,1) == 5 % SURFACE
        box_color(1,1:3) =  [1 1 0];
    end
    if ethogram(1,1) == 6 % SURFACE
        box_color(1,1:3) =  [1 1 0];
    end
end


end

