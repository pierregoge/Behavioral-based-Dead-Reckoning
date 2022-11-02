function [J] = var_jerk(var,var1,var2,var3)

if nargin < 2
    
    if isstruct(var)
        J = var.A;
        if ~isfield(var.Ad,'data')
            fprintf('var_odba: input must be a proper sensor structure with dynamic acceleation data \n') ;
            return
        end     
        if ~strcmp(var.Ad.sampling,'regular')
            fprintf('var_odba: input must be a regularly sampled sensor structure\n') ;
            return
        end
        
        j =  njerk(var.A) ;
    else
        fprintf('var_odba: If 1 input need to a strcuture with dynamic acceleration array \n') ;
        return
    end
    
else
    a(:,1) = var;
    a(:,2) = var1;
    a(:,3) = var2;
    fs = var3;
    J = njerk(a,fs);
end

if isstruct(var.A)
    J.data = j;
    J.name = 'J';
    J.full_name= 'Jerk';
end



end