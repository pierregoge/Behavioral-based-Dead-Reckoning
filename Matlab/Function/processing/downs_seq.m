function      [seq_out] = downs_seq(seq,df)

%     seq_out = downs_seq(seq,df)
%     Reduce the sampling rate of sequence sctructure by an integer factor.
%     It is a simple downsampling without average or other function.
%	  This is similar to downsample() but made for a sequence sctructure. Each
%	  component of the seq.var are downsampled
%
%		Inputs:
%		seq is a sequence structure containing var field for different
%		variable or sensor
%		df is the decimation factor. The output sampling rate is the input
%		sampling rate divided by df. df must be an integer greater than 1.
%
%		Returns:
%		seq_out is the downsampled sequence structure.
%
%       Downsample is performed for the data of every field of seq.var. The
%       other filed as sampling frequency or computing history of the data
%       are update
%
%     TODO : Function to select which variable to downsample. Adding
%     Behavior downsampling
%
%     pierregogendeau@gmail.com
%     last modified: 14/03/2022

if nargin<2
    help downs_seq ;
    return
end

if round(df)~=df
    df = round(df) ;
    fprintf('downs_seq: needs integer decimation factor. Using %d\n',df) ;
end

if isstruct(seq)
    if ~isfield(seq,'var')
        fprintf('downs_seq: input sctructure need var array\n') ;
        return
    end
end

seq_out = seq;

Z = fieldnames(seq.var);  % Number and name of all the variable field of the sequence

for i=1:length(Z)   % For loop the compute each variable field
    
    
    clear f
    X = seq.var;
    [X.(Z{i})] = getfield(X,Z{i});
    
    if isstruct([X.(Z{i})])
        
        if ~strcmp(X.(Z{i}).name,'B')
            
            if ~isfield(X.(Z{i}),'data')
                fprintf('downs_seq: input must be a proper sensor structure\n') ;
                return
            end
            
            if ~strcmp(X.(Z{i}).sampling,'regular')
                fprintf('downs_seq: input must be a regularly sampled sensor structure\n') ;
                return
            end
            data = X.(Z{i}).data(:,:) ;
            
            y = downsample(data,df);    % Simple downsampling function
            
            % Store and update the result of the downsampling in the output sequence
            if exist('X','var')
                X.(Z{i}).data = y ;
                X.(Z{i}).sampling_rate = X.(Z{i}).sampling_rate/df ;
                h = sprintf('downs_seq(%d)',df) ;
                if ~isfield(X.(Z{i}),'history') || isempty(X.(Z{i}).history)
                    X.(Z{i}).history = h ;
                else
                    X.(Z{i}).history = [X.(Z{i}).history ',' h] ;
                end
                
                seq_out.var.(Z{i}) = X.(Z{i}) ;
                
            end
            
        else
            data = X.(Z{i}).data ;
            y_b(:,1) = data(:,1);
            y_b(:,2:3) = round(data(:,2:3) ./ df);
            y_b(:,2) = y_b(:,2)+1;
            
            % Store and update the result of the downsampling in the output sequence
            if exist('X','var')
                X.(Z{i}).data = y_b ;
                h = sprintf('downs_seq(%d)',df) ;
                if ~isfield(X.(Z{i}),'history') || isempty(X.(Z{i}).history)
                    X.(Z{i}).history = h ;
                else
                    X.(Z{i}).history = [X.(Z{i}).history ',' h] ;
                end
                
                seq_out.var.(Z{i}) = X.(Z{i}) ;
                
            end
        end
        
    end
    
end

seq_out.fs_max = seq_out.var.A.sampling_rate;

end

