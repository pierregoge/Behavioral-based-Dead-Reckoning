function [data,data_ref,diff_traj] = seq2data_behav_analyse(seq,seq_ref,v,behavior)


if v == 1
    
    len_etho = length(seq.var.B.data(:,1));
    inc1 = 1;
    inc = 0;
    
    for j=1:len_etho
        if seq.var.B.data(j,1) ~= behavior
            inc = inc+1;
        end
    end
    
    if inc == len_etho
        data = [];
        data_ref= [];
        return
        
    end
    
    first_flag = 1;
    
    
    for i=1:len_etho
        
        
        if seq.var.B.data(i,1) == behavior
            
            lim_inf = seq.var.B.data(i,2);
            lim_sup = seq.var.B.data(i,3);
            
            x_ref =  -seq_ref.var.Y.data;
            y_ref =  -seq_ref.var.X.data;
            x =  seq.var.X.data;
            y =  seq.var.Y.data;
            %mean_pitch = mean(seq.var.Pi.data(lim_inf:lim_sup));               % Mean pitch
            
            if first_flag == 1
                
                data1(1:lim_sup-lim_inf,1) =  seq.var.S.data(lim_inf+1:lim_sup);
                data2(1:lim_sup-lim_inf,1) =  seq.var.X.data(lim_inf+1:lim_sup);
                data3(1:lim_sup-lim_inf,1) =  seq.var.Y.data(lim_inf+1:lim_sup);
                
                for j=lim_inf+1:lim_sup
                    
                    data4(j-lim_inf,1) =  (x_ref(j,1)-x_ref(j-1,1)) - (x(j,1)-x(j-1,1));
                    data5(j-lim_inf,1) =  (y_ref(j,1)-y_ref(j-1,1)) - (y(j,1)-y(j-1,1));
                end
                
                data_ref1(1:lim_sup-lim_inf,1) =  seq_ref.var.S.data.Var1(lim_inf+1:lim_sup);
                data_ref2(1:lim_sup-lim_inf,1) =  -seq_ref.var.Y.data(lim_inf+1:lim_sup);
                data_ref3(1:lim_sup-lim_inf,1) =  -seq_ref.var.X.data(lim_inf+1:lim_sup);
                first_flag = 0;
            else
                data1 =  [data1(:,1); seq.var.S.data(lim_inf+1:lim_sup)];
                data2 =  [data2(:,1); seq.var.X.data(lim_inf+1:lim_sup)];
                data3 =  [data3(:,1); seq.var.Y.data(lim_inf+1:lim_sup)];
                
                for j=lim_inf+1:lim_sup
                    data4 =  [data4(:,1); (x_ref(j,1)-x_ref(j-1,1)) - (x(j,1)-x(j-1,1))];
                    data5 =  [data5(:,1); (y_ref(j,1)-y_ref(j-1,1)) - (y(j,1)-y(j-1,1))];
                end
                
                data_ref1 =  [data_ref1(:,1); seq_ref.var.S.data.Var1(lim_inf+1:lim_sup)];
                data_ref2 =  [data_ref2(:,1); seq_ref.var.X.data(lim_inf+1:lim_sup)];
                data_ref3 =  [data_ref3(:,1); seq_ref.var.Y.data(lim_inf+1:lim_sup)];
            end
            
            
        end
        
        
        
    end
    
    
    
    
    data = [data1 , data2, data3];
    diff_traj = [data4(1:end-1,1), data5(1:end-1,1)];
    data_ref = [data_ref1 , data_ref2,data_ref3];
    
    
    
end


end

