function    seq_out = merge_seq(seq1,seq2)
%
%     seq_out = merge_seq(seq1,seq2)
%     Merge the fields of minimum 2 sequence sctructure. It doesn't merge B
%     field
%
%		Inputs:
%		seq1 is a sequence strcuture to merge
%       seq2 is a sequence strcuture to merge
%
%		Returns:
%		seq_out is a structure containing all the merge fields in s1 and
%		s2. The type is 'merge_seq' without strarting and ending time
%
%     pierregogendeau@gmail.com
%     Last modified: 15/03/2022

if nargin < 2
    fprintf('merge_seq : Need at least 2 sequence\n') ;
    help merge_seq
    return
end

if ~isstruct(seq1) || ~isstruct(seq2)
    fprintf('merge_seq : Input seq need to be sctruct \n') ;
    return
else
    if ~isfield(seq1,'var')
        fprintf('merge_seq : Need to be a appropriate sequence with var field \n') ;
        return
    end
    b = 'B';
     if isfield(seq1.var,b)
        seq1.var = rmfield(seq1.var,b);
     end
    if isfield(seq2.var,b)
        seq2.var = rmfield(seq2.var,b);
    end
end



Z1 = fieldnames(seq1.var);   %Store the field name of the variable
Z2 = fieldnames(seq2.var);   %Store the field name of the variable

if length(Z1) ~= length(Z2)
    fprintf('merge_seq : Input sequences need to have the same number of field \n') ;
    return
end
for i=1:length(Z1)   % For loop the compute each variable field
    if ~strcmp(Z1(i,1),Z2(i,1))
        fprintf('merge_seq : Input sequences need to have the same field \n') ;
        return
    end
end

for i=1:length(Z1)   % For loop the compute each variable field
    
    X1 = seq1.var;
    X2 = seq2.var;
    
    [X1.(Z1{i})] = getfield(X1,Z1{i});
    [X2.(Z2{i})] = getfield(X2,Z2{i});
    
    if (isstruct([X1.(Z1{i})]) || isstruct([X2.(Z2{i})]) ) && ~strcmp(([X1.(Z1{i}).name]),'B')  %Check if all the field are structure. Does not merge B field
        
        X.(Z1{i}) = X1.(Z1{i}); % Copy the var field in the new sequence
        
        if ~isfield(X1.(Z1{i}),'data') || ~isfield(X1.(Z1{i}),'data')
            fprintf('merge_seq: input var must be a proper sensor structure\n') ;
            return
        end
        if ~isfield(X1.(Z1{i}),'sampling_rate') || ~isfield(X1.(Z1{i}),'sampling_rate')
            fprintf('merge_seq: input var must have sampling rate field\n') ;
            return
        else 
            if X1.(Z1{i}).sampling_rate ~= X2.(Z2{i}).sampling_rate
            fprintf('merge_seq: input var must have the same sampling rate\n') ;
            return
            end
        end
       
        data1 = X1.(Z1{i}).data ;
        data2 = X2.(Z2{i}).data ;
        
        y = [data1; data2];    % Simple merge function
        
        % Store and update the result of the merge in the output sequence
        if exist('X1','var')
            X.(Z1{i}).data = y ;
            h = sprintf('merge_seq') ;
            if ~isfield(X.(Z1{i}),'history') || isempty(X.(Z1{i}).history)
                X.(Z1{i}).history = h ;
            else
                X.(Z1{i}).history = [X.(Z1{i}).history ',' h] ;
            end
            
            % The function doesn't give start and and time because it can
            % be not followed sequences
            seq_out.var.(Z1{i}) = X.(Z1{i}) ;
            %seq_out.fs = seq1.fs_max ;
            seq_out.type = 'merge_seq';
            
        end
        
    end
    
end


end
