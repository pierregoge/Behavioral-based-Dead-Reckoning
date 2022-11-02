function [data_filter] = mov_mean_filter(coeff,x)




if isstruct(x)
	X = x ;
	if ~isfield(x,'data')
		fprintf('mov_mean_filter: input must be a proper sensor structure\n') ;
		return
	end
	if ~strcmp(x.sampling,'regular')
		fprintf('mov_mean_filter: input must be a regularly sampled sensor structure\n') ;
		return
	end
	x = x.data ;
end


conv_coeff = (2*coeff+1);
buf = 0;
for i =coeff+1:length(x(:,1))-coeff
    
    for j = i-coeff:i+coeff
        buf = buf + x(j,:);
    end
    data_filter(i,:) = buf/conv_coeff;
    buf = 0;
       
end

for i =length(x(:,1))-coeff:length(x(:,1))
    data_filter(i,:) = x(i,:);
end

for i =1:coeff
    data_filter(i,:) = x(i,:);
end

if exist('X','var')
	X.data = data_filter ;
	%X.sampling_rate = X.sampling_rate/df ;
	h = sprintf('mov_mean_filter(%d)',coeff) ;
	if ~isfield(X,'history') || isempty(X.history)
		X.history = h ;
	else
		X.history = [X.history ',' h] ;
	end
	data_filter = X ;
end
	

end

