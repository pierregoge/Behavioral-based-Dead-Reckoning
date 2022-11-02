function [O] = var_odba(var,var1,var2)

if nargin < 2
    
    if isstruct(var)
        O = var.A;
        if ~isfield(var.Ad,'data')
            fprintf('var_odba: input must be a proper sensor structure with dynamic acceleation data \n') ;
            return
        end     
        if ~strcmp(var.Ad.sampling,'regular')
            fprintf('var_odba: input must be a regularly sampled sensor structure\n') ;
            return
        end
        x = var.Ad.data(:,1) ;
        y = var.Ad.data(:,2);
        z = var.Ad.data(:,3);
    else
        fprintf('var_odba: If 1 input need to a strcuture with dynamic acceleration array \n') ;
        return
    end
    
else
    x = var;
    y = var1;
    z = var2;
end

o = sqrt(x.^2+y.^2+z.^2) ;

if isstruct(var.A)
    O.data = o;
    O.name = 'O';
    O.full_name= 'ODBA';
end


end


