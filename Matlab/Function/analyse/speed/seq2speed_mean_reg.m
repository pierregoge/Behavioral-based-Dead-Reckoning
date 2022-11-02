function [y,xtrain] = seq2speed_mean_reg(seq,seq_ref,v,behavior)


if v == 1
    
    len_etho = length(seq.var.B.data(:,1));
    inc1 = 1;
    inc = 0;
    y_buf = [];
    len_buf = 0;
    
    for j=1:len_etho
        if seq.var.B.data(j,1) ~= behavior
            inc = inc+1;
        end
    end
    
    if inc == len_etho
        y = [];
        xtrain= [];
        return
        
    end
    
    
    first_behav = 1;
    
    for i=1:len_etho
        
        
        if seq.var.B.data(i,1) == behavior
            lim_inf = seq.var.B.data(i,2);
            lim_sup = seq.var.B.data(i,3)-1;
            
            mean_pitch = mean(seq.var.Pi.data(lim_inf:lim_sup,1));               % Mean pitch
            
            [nb_str_x, avg_str_x] = stroke_nb(seq.var.Ad.data(lim_inf:lim_sup,1));
            [nb_str_z, avg_str_z] = stroke_nb(seq.var.Ad.data(lim_inf:lim_sup,3));
            
            len = length(lim_inf:lim_sup);
            
            if first_behav == 1
                
                if seq.var.Pi.sampling_rate == 1
                    y(1:lim_sup-lim_inf+1,1,1) = mean(seq_ref.var.S.data.Var1(lim_inf:lim_sup,1));
                else
                    y(1:lim_sup-lim_inf+1,1,1) = mean(seq_ref.var.S10.data.Var1(lim_inf:lim_sup,1));
                end
                
                xtrain1(1:lim_sup-lim_inf+1,1) =  mean(seq.var.Pi.data(lim_inf:lim_sup,1));               % Mean pitch
                xtrain2(1:lim_sup-lim_inf+1,1) =  seq.var.P.data(lim_sup) - seq.var.P.data(lim_inf);    % Difference of depth
                xtrain3(1:lim_sup-lim_inf+1,1) =  lim_sup-lim_inf;                                      % Length of behavior in second
                xtrain4(1:lim_sup-lim_inf+1,1) =  mean(seq.var.P.data(lim_inf:lim_sup,1));                % mean depth
                xtrain5(1:lim_sup-lim_inf+1,1) =  (xtrain2(end,1)/(tan(xtrain1(end,1)))/xtrain3(end,1));
                xtrain6(1:lim_sup-lim_inf+1,1) =  nb_str_x;               % Mean ODBA
                xtrain7(1:lim_sup-lim_inf+1,1) =  avg_str_x;               % Mean Acc
                xtrain8(1:lim_sup-lim_inf+1,1) =  nb_str_z;               % Mean ODBA
                xtrain9(1:lim_sup-lim_inf+1,1) =  avg_str_z;               % Mean Acc
                %                 y_buf = [y_buf; seq_ref.var.S.data.Var1(lim_inf:lim_sup,1)];
                %                 len_buf = len_buf + len;
                
                first_behav = 0;
                
            else
                if seq.var.Pi.sampling_rate == 1
                    buf(1:lim_sup-lim_inf+1,1) = mean(seq_ref.var.S.data.Var1(lim_inf:lim_sup,1));
                    y = [y; buf];
                else
                    buf(1:lim_sup-lim_inf+1,1) = mean(seq_ref.var.S10.data.Var1(lim_inf:lim_sup,1));
                    y = [y; buf];
                end
                
                buf1(1:lim_sup-lim_inf+1,1) = mean(seq.var.Pi.data(lim_inf:lim_sup,1));
                xtrain1 =  [xtrain1; buf1];               % Mean pitch
                buf2(1:lim_sup-lim_inf+1,1) = seq.var.P.data(lim_sup) - seq.var.P.data(lim_inf);
                xtrain2 =  [xtrain2; buf2];    % Difference of depth
                buf3(1:lim_sup-lim_inf+1,1) = lim_sup-lim_inf;
                xtrain3 =  [xtrain3; buf3];% Length of behavior in second
                buf4(1:lim_sup-lim_inf+1,1) = mean(seq.var.P.data(lim_inf:lim_sup,1));
                xtrain4 =  [xtrain4; buf4];                % mean depth
                buf5(1:lim_sup-lim_inf+1,1) = (xtrain2(end,1)/(sin(xtrain1(end,1)))/xtrain3(end,1));
                xtrain5 =  [xtrain5; buf5 ];
                buf6(1:lim_sup-lim_inf+1,1) = nb_str_x;
                xtrain6 =  [xtrain6; buf6];               % Mean ODBA
                buf7(1:lim_sup-lim_inf+1,1) = avg_str_x;
                xtrain7 =  [xtrain7; buf7];               % Mean Acc
                buf8(1:lim_sup-lim_inf+1,1) = nb_str_z;
                xtrain8 =  [xtrain8; buf8];               % Mean ODBA
                buf9(1:lim_sup-lim_inf+1,1) =  avg_str_z;
                xtrain9 =  [xtrain9; buf9];               % Mean Acc
                
                clear buf buf1 buf2 buf3 buf4 buf5 buf6 buf7 buf8 buf9
%                 y_buf = [y_buf; seq_ref.var.S.data.Var1(lim_inf:lim_sup,1)];
%                 len_buf = len_buf + len;
                
%                 inc1 = inc1+1;
            end
            
            xtrain = [xtrain1, xtrain2, xtrain3, xtrain4, xtrain5, xtrain6, xtrain7, xtrain8, xtrain9];
            
            
        end
        
    end
    
    
    
end


end

