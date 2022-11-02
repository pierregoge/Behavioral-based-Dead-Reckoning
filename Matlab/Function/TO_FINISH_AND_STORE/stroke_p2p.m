function [p2p,min_out,max_out] = stroke_p2p(x,t)

max = 0;
min = 0;
% 
% if exist(t)
len = length(t(:,1));

for j=1:t(1,1)
    if x(j,1) > max
        max = x(j,1);
    end
    if x(j,1) < min
        min = x(j,1);
    end
end
p2p = abs(max)+abs(min);
max = 0;
min = 0;


for i=2:len
    for j=t(i-1,1):t(i,1)
        if x(j,1) > max
            max = x(j,1);
        end
        if x(j,1) < min
            min = x(j,1);
        end
    end
    p2p(i,1) = abs(max)+abs(min);
    min_out(i,1) = min;
    max_out(i,1) = max;
    max = 0;
    min = 0;
end

% else
%   p2p = 0;
% end

end

