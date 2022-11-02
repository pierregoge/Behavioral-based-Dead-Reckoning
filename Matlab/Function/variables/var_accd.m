function [Ad] = var_accd(var,var1)


if nargin < 2
    
    if isstruct(var.A)
        Ad = var.A;
        if ~isfield(var.A,'data')
            fprintf('var_accd: input must be a proper sensor structure with raw acceleation data \n') ;
            return
        end
        if ~isfield(var.Ag,'data')
            fprintf('var_accd: input must be a proper sensor structure with gravitationnal acceleation data \n') ;
            return
        end
        if ~strcmp(var.A.sampling,'regular')
            fprintf('var_accd: input must be a regularly sampled sensor structure\n') ;
            return
        end
        x = var.A.data ;
        x1 = var.Ag.data;
    else
        fprintf('var_accd: Need in input variable raw and gravitationnal acceleration \n') ;
        return
    end
    
else
    x = var;
    x1 = var1;
end


y = x - x1;

if isstruct(var.A)
    Ad.data = y;
end


end