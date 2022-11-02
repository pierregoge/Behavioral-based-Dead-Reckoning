function [T] = zero_cross(x)

T = [];
len = length(x(:,1));
sign = 0;
flag_pos = 0;
flag_neg = 0;
sign_threshold = 1;
inc = 1;

if isempty(x)
    x = 0;
end

if x(1,1) >= 0 
    sign = 1;
else
    sign = -1;
end

for i=2:len
    
    if sign == 1 && x(i,1) < 0
        flag_neg = flag_neg+1;
        %if flag_neg >= sign_threshold
            sign = -1;
            flag_neg = 0;
            T(inc,1) = i;%-sign_threshold;
            inc = inc+1;
        %end
    end
    
    if sign == -1 && x(i,1) > 0
        flag_pos = flag_pos+1;
        %if flag_pos >= sign_threshold
            sign = 1;
            flag_pos = 0;
            %T(inc,1) = i-sign_threshold;
            %inc = inc+1;
        %end
    end
    
end

end

