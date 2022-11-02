function [seq_out] = crop_seq(seq,lim_inf,lim_sup)

%     seq_out = crop_seq(seq,lim_inf,lim_sup)
%     Crop a sequence sctructure or variable sctructure between two limits.
%
%		Inputs:
%		seq is a sequence structure containing var field for different
%		variable or sensor or var sensor structure
%		lim_inf is the inferior limit of the crop in second
%       lim_sup is the superior limit of the crop in second
%
%		Returns:
%		seq_out is crop sequence structure.
%
%       It allows to cut behavior data array if the input is a sequence
%       with behavior.
%
%
%     pierregogendeau@gmail.com
%     last modified: 14/03/2022

if nargin<3
    help crop_seq ;
    return
end


if isstruct(seq) && isfield(seq,'data')  %Check if the input is a data structure
    
    seq_out = seq;
    
    if lim_sup > length(seq.data(:,1))/seq.sampling_rate
        fprintf('crop_seq: Superior crop limit is superior than number of variable \n');
        return
    end
    
    
    if ~isfield(seq,'data')
        fprintf('crop_seq: input must be a proper sensor structure\n') ;
        return
    end
    if ~strcmp(seq.sampling,'regular')
        fprintf('downs_seq: input must be a regularly sampled sensor structure\n') ;
        return
    end
    
    data = seq.data ;
    fs = seq.sampling_rate;
    
    crop_len = length(lim_inf:lim_sup)-1;
    start_crop = (lim_inf)*fs;
    end_crop =  (crop_len)*fs;
    
    y =data(lim_inf:lim_sup,:);    % Simple downsampling function
    
    seq_out.data = y;
    
end



if isstruct(seq) && isfield(seq,'var')  %Check if the input is a sequence structure
    
%     if lim_inf < seq.tstart
%         fprintf('crop_seq: Inferior crop limit is inferior than starting time \n');
%         return
%     end
%     if lim_sup > seq.tend
%         fprintf('crop_seq: Superior crop limit is superior than ending time \n');
%         return
%     end
    
    seq_out = seq;
    
    Z = fieldnames(seq.var);  % Number and name of all the variable field of the sequence
    
    for i=1:length(Z)   % For loop the compute each variable field
        
        clear f
        X = seq.var;
        [X.(Z{i})] = getfield(X,Z{i});
        
        
        if isstruct([X.(Z{i})]) && ~strcmp(([X.(Z{i}).name]),'B') && ~strcmp(([X.(Z{i}).name]),'X') && ~strcmp(([X.(Z{i}).name]),'Y')
            
            if ~isfield(X.(Z{i}),'data')
                fprintf('crop_seq: input must be a proper sensor structure\n') ;
                return
            end
%             if ~strcmp(X.(Z{i}).sampling,'regular')
%                 fprintf('crop_seq: input must be a regularly sampled sensor structure\n') ;
%                 return
%             end
            data = X.(Z{i}).data ;
            fs = X.(Z{i}).sampling_rate;
            
            
            crop_len = length(lim_inf:lim_sup);
            start_crop = seq.tstart*fs+(lim_inf)*fs;
            end_crop =  seq.tstart*fs+(lim_inf+crop_len)*fs-1*fs-1;
            
            y =data(lim_inf*fs:(lim_sup)*fs-1,:);    % Simple downsampling function
            
            % Store and update the result of the downsampling in the output sequence
            if exist('X','var')
                X.(Z{i}).data = y ;
                h = sprintf('crop_seq') ;
                if ~isfield(X.(Z{i}),'history') || isempty(X.(Z{i}).history)
                    X.(Z{i}).history = h ;
                else
                    X.(Z{i}).history = [X.(Z{i}).history ',' h] ;
                end
                
                seq_out.var.(Z{i}) = X.(Z{i}) ;
                
            end
            
        elseif isstruct([X.(Z{i})]) && strcmp(([X.(Z{i}).name]),'B')   % If Behavior data are available crop is different
         
         data_etho = X.(Z{i}).data;
         
         for j=1:length(data_etho(:,1))
         
             if start_crop <= data_etho(j,3) && start_crop > data_etho(j,2)
                 lim_inf_e = j;
                 %start_b = (lim_inf*fs)-data_etho(j,3);
             end
             if end_crop >= data_etho(j,2) && end_crop < data_etho(j,3) 
                 lim_sup_e = j;
                 %end_b = (lim_inf*fs-1 - data_etho(j,3));
             end               
         end
         
         y = data_etho(lim_inf_e:lim_sup_e,:);
             
         %y(1,2) = y(1,2)+start_b;
         y(:,2:3) = y(:,2:3)-start_crop;
         y(end,3) = end_crop-start_crop;
         y(1,2) = 1;
         % Store and update the result of the downsampling in the output sequence
            if exist('X','var')
                X.(Z{i}).data = y ;
                h = sprintf('crop_seq') ;
                if ~isfield(X.(Z{i}),'history') || isempty(X.(Z{i}).history)
                    X.(Z{i}).history = h ;
                else
                    X.(Z{i}).history = [X.(Z{i}).history ',' h] ;
                end
                
                seq_out.var.(Z{i}) = X.(Z{i}) ;
                
            end
       
         end

        if isstruct([X.(Z{i})]) && (strcmp(([X.(Z{i}).name]),'X') || strcmp(([X.(Z{i}).name]),'Y'))  % If X and Y  data are available crop is different
            
            data = X.(Z{i}).data;
            
            y = data(lim_inf:lim_sup,1)-data(lim_inf,1);  % Track already compute will start a 0m
            
             X.(Z{i}).data = y ;
             seq_out.var.(Z{i}) = X.(Z{i}) ;
              
        end
    end
    
    seq_out.tstart = lim_inf;
    seq_out.tend = lim_sup;
    
end


end

