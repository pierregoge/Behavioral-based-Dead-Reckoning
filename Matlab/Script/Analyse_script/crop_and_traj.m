%% Crop seq to match reference seq and calculate trajectories

for j=1:4
    
    buf_in = struct(seq(j,1));
    
    % Crop sequence with ref sequence limits
    for i=1:length(seq_s_ref(:,1))
        lim_inf = seq_s_ref(i,1).tstart;
        lim_sup = seq_s_ref(i,1).tend-1;
        [buf_in_s(i,j)] = crop_seq(buf_in,lim_inf,lim_sup);
    end
    
    % Calculate trajectory
    offset_h = -19.32;%-19.32;
    for i=1:length(seq_s_ref(:,1))
        [seq_s(i,j)] = seq_seq2htrack(buf_in_s(i,j),offset_h);    
    end
end

%
j=5;
for i=1:length(seq_s_ref(:,1))
        lim_inf = seq_s_ref(i,1).tstart;
        lim_sup = seq_s_ref(i,1).tend-1;
        [buf_in_s(i,j)] = crop_seq(buf_in,lim_inf,lim_sup);
        
        buf_in_s(i,j).var.S.data = seq_s_ref(i,1).var.S.data.Var1(1:end-2);
        
        [seq_s(i,j)] = seq_seq2htrack(buf_in_s(i,j),offset_h);    
end


[seq] = seq_seq2htrack(seq,offset_h);
clear T buf_in buf_in_s

 