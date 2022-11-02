%% Concat variable for analyse

% Variable
s_seq_ref = [];
s_seq = [];
x = [];
x_ref =  [];
y = [];
y_ref =  [];

for i=start_s:end_s
    
    if i ~= 6 %Remove 6th sequence too noisy (see plot)
        
    s_ref_buf = seq_s_ref(i).var.S.data.Var1(1:end-2,1);
    x_ref_buf = -seq_s_ref(i).var.Y.data(1:end-2,1);
    y_ref_buf = -seq_s_ref(i).var.X.data(1:end-2,1);
    
    s_seq_ref = [s_seq_ref; s_ref_buf];
    x_ref = [x_ref; x_ref_buf];
    y_ref = [y_ref; y_ref_buf];
    
    for j=1:5
        if j ==1
            s_buf(:,j) = seq_s(i,j).var.S.data(:,1);
            y_buf(:,j) = seq_s(i,j).var.Y.data(:,1);
            x_buf(:,j) = seq_s(i,j).var.X.data(:,1);
        elseif j==3
            s_buf(:,j) = seq_s(i,j).var.S.data(:,2);
            y_buf(:,j) = seq_s(i,j).var.Y.data(:,1);
            x_buf(:,j) = seq_s(i,j).var.X.data(:,1);
        elseif j==2
            s_buf(:,j) = seq_s(i,j).var.S.data(:,2);
            y_buf(:,j) = seq_s(i,j).var.Y.data(:,1);
            x_buf(:,j) = seq_s(i,j).var.X.data(:,1);
        else
            s_buf(:,j) = seq_s(i,j).var.S.data(:,1);
            y_buf(:,j) = seq_s(i,j).var.Y.data(:,1);
            x_buf(:,j) = seq_s(i,j).var.X.data(:,1);
        end
        
    end
   
    
    y = [y; y_buf];
    x = [x; x_buf];
    s_seq = [s_seq; s_buf];
    clear s_buf x_buf y_buf
    
    end
    
end

clear x_ref_buf y_ref_buf s_ref_buf
