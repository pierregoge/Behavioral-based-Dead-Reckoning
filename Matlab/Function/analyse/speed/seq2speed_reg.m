function [y,xtrain] = seq2speed_reg(seq,seq_ref,v,behavior)


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
        y = [];
        xtrain= [];
        return
        
    end
    
    first_flag = 1;
    
    
    
    for i=1:len_etho
        
        
        if seq.var.B.data(i,1) == behavior
            
            lim_inf = seq.var.B.data(i,2);
            lim_sup = seq.var.B.data(i,3);
            %mean_pitch = mean(seq.var.Pi.data(lim_inf:lim_sup));               % Mean pitch
            
            if first_flag == 1
                coeff_smooth = seq.var.O.sampling_rate*5;
                y(1:lim_sup-lim_inf+1,1) = seq_ref.var.S10.data.Var1(lim_inf:lim_sup);
                xtrain1(1:lim_sup-lim_inf+1,1) =  seq.var.Pi.data(lim_inf:lim_sup);               % pitch
                xtrain2(1:lim_sup-lim_inf+1,1) =  smoothdata(seq.var.Pi.data(lim_inf:lim_sup),'movmean',coeff_smooth);  % smooth Pitch
                xtrain3(1:lim_sup-lim_inf+1,1) =  seq.var.P.data(lim_inf:lim_sup);
                xtrain4(1:lim_sup-lim_inf+1,1) =  seq.var.O.data(lim_inf:lim_sup);          % ODBA
                xtrain5(1:lim_sup-lim_inf+1,1) =  smoothdata(seq.var.O.data(lim_inf:lim_sup),'movmean',coeff_smooth);
                %                     data1(1:lim_sup-lim_inf+1,1) =  seq.var.X.data(lim_inf:lim_sup);
                %                     data2(1:lim_sup-lim_inf+1,1) =  seq.var.Y.data(lim_inf:lim_sup);          % ODBA
                %                     data_ref1(1:lim_sup-lim_inf+1,1) =  seq_ref.var.X.data(lim_inf:lim_sup);
                %                     data_ref2(1:lim_sup-lim_inf+1,1) =  seq_ref.var.Y.data(lim_inf:lim_sup);          % ODBA
                %xtrain4(1:lim_sup-lim_inf+1,1) =  seq.var.O.data(lim_inf:lim_sup);          % ODBA% smooth ODBA
                first_flag = 0;
            else
                y = [y(:,1) ; seq_ref.var.S10.data.Var1(lim_inf:lim_sup,1)];
                xtrain1 =  [xtrain1(:,1); seq.var.Pi.data(lim_inf:lim_sup)];               % pitch
                xtrain2 =  [xtrain2(:,1); smoothdata(seq.var.Pi.data(lim_inf:lim_sup),'movmean',coeff_smooth)];  % smooth Pitch
                xtrain3 =  [xtrain3(:,1);seq.var.P.data(lim_inf:lim_sup)];
                xtrain4 =  [xtrain4(:,1);seq.var.O.data(lim_inf:lim_sup)];          % ODBA
                xtrain5 =  [xtrain5(:,1);smoothdata(seq.var.O.data(lim_inf:lim_sup),'movmean',coeff_smooth)];          % smooth ODBA
                %                     data1 =  [data1(:,1); seq.var.X.data(lim_inf:lim_sup)];
                %                     data2 =  [data2(:,1); seq.var.Y.data(lim_inf:lim_sup)];          % ODBA
                %                     data_ref1 =  [data_ref1(:,1); seq_ref.var.X.data(lim_inf:lim_sup)];
                %                     data_ref2 =  [data_ref2(:,1); seq_ref.var.Y.data(lim_inf:lim_sup)];          % ODBA
            end
            
            
        end
        
        
        
    end
    
    
    
    
    xtrain = [ xtrain1 , xtrain2 , xtrain3, xtrain4 , xtrain5];
    
    
    
end

if v == 3
    
    len_etho = length(seq.var.B2.data(:,1));
    inc1 = 1;
    inc = 0;
    
    
    y = [];
    xtrain= [];
    xtrain1= [];
    xtrain2= [];
    xtrain3= [];
    xtrain4= [];
    xtrain5= [];
    xtrain6= [];
    xtrain7= [];
    
     
    for i=1:len_etho
        
        %seq.var.B2.data(i,2) == 1 &&
        if  (seq.var.B2.data(i,1) == behavior(1,1) || seq.var.B2.data(i,1) == behavior(2,1) || seq.var.B2.data(i,1) == behavior(3,1) || seq.var.B2.data(i,1) == behavior(4,1) || seq.var.B2.data(i,1) == behavior(5,1) || seq.var.B2.data(i,1) == behavior(6,1))
            
            coeff_smooth = seq.var.O.sampling_rate*5;
            y = [y ; seq_ref.var.S.data.Var1(i,1)];
            xtrain1 =  [xtrain1; seq.var.Pi.data(i,1)];               % pitch
            %xtrain2 =  [xtrain2; smooth_pi];  % smooth Pitch
            xtrain3 =  [xtrain3;seq.var.P.data(i,1)];
            xtrain4 =  [xtrain4;seq.var.O.data(i,1)];          % ODBA
            %xtrain5 =  [xtrain5;smooth_o];          % smooth ODBA
            xtrain6 =  [xtrain6;seq.var.B2.data(i,4)];    % p2p stroke      
            xtrain7 =  [xtrain7;seq.var.B2.data(i,3)];    % Stroke freq 
            %                     data1 =  [data1(:,1); seq.var.X.data(lim_inf:lim_sup)];
            %                     data2 =  [data2(:,1); seq.var.Y.data(lim_inf:lim_sup)];          % ODBA
            %                     data_ref1 =  [data_ref1(:,1); seq_ref.var.X.data(lim_inf:lim_sup)];
            %                     data_ref2 =  [data_ref2(:,1); seq_ref.var.Y.data(lim_inf:lim_sup)];          % ODBA
        end
        
        
    end
    
    
xtrain2 =  smoothdata(xtrain1,'movmean',coeff_smooth);  % smooth Pitch
xtrain5 =  smoothdata(xtrain4,'movmean',coeff_smooth);

%xtrain = [ xtrain1 , xtrain2 , xtrain3, xtrain4 , xtrain5, xtrain6 , xtrain7];
xtrain  = xtrain5;

end

if v == 4
    
    len_etho = length(seq.var.B2.data(:,1));
    inc1 = 1;
    inc = 0;
    
    
    y = [];
    xtrain= [];
    xtrain1= [];
    xtrain2= [];
    xtrain3= [];
    xtrain4= [];
    xtrain5= [];
    xtrain6= [];
    xtrain7= [];
    
     
    for i=1:len_etho
        
        
        if  seq.var.B2.data(i,2) == 1 && (seq.var.B2.data(i,1) == behavior(1,1) || seq.var.B2.data(i,1) == behavior(2,1) || seq.var.B2.data(i,1) == behavior(3,1) )
            
            coeff_smooth = seq.var.O.sampling_rate*5;
            y = [y ; seq_ref.var.S.data.Var1(i,1)];
            xtrain1 =  [xtrain1; seq.var.Pi.data(i,1)];               % pitch
            %xtrain2 =  [xtrain2; smooth_pi];  % smooth Pitch
            xtrain3 =  [xtrain3;seq.var.P.data(i,1)];
            xtrain4 =  [xtrain4;seq.var.O.data(i,1)];          % ODBA
            %xtrain5 =  [xtrain5;smooth_o];          % smooth ODBA
            xtrain6 =  [xtrain6;seq.var.B2.data(i,4)];    % p2p stroke      
            xtrain7 =  [xtrain7;seq.var.B2.data(i,3)];    % Stroke freq 
            %                     data1 =  [data1(:,1); seq.var.X.data(lim_inf:lim_sup)];
            %                     data2 =  [data2(:,1); seq.var.Y.data(lim_inf:lim_sup)];          % ODBA
            %                     data_ref1 =  [data_ref1(:,1); seq_ref.var.X.data(lim_inf:lim_sup)];
            %                     data_ref2 =  [data_ref2(:,1); seq_ref.var.Y.data(lim_inf:lim_sup)];          % ODBA
        end
        
        
    end
    
    
xtrain2 =  smoothdata(xtrain1,'movmean',coeff_smooth);  % smooth Pitch
xtrain5 =  smoothdata(xtrain4,'movmean',coeff_smooth);

xtrain = [ xtrain1 , xtrain2 , xtrain3, xtrain4 , xtrain5, xtrain6 , xtrain7];
    
end


if v == 5
    
    len_etho = length(seq.var.B2.data(:,1));
    inc1 = 1;
    inc = 0;
    
    
    y = [];
    xtrain= [];
    xtrain1= [];
    xtrain2= [];
    xtrain3= [];
    xtrain4= [];
    xtrain5= [];
    xtrain6= [];
    xtrain7= [];
    
     
    for i=1:len_etho
        
        %seq.var.B2.data(i,2) == 1 &&
        if  (seq.var.B2.data(i,1) == behavior(1,1) && seq.var.B2.data(i,2) == behavior(2,1))
            
            coeff_smooth = seq.var.O.sampling_rate*5;
            y = [y ; seq_ref.var.S.data.Var1(i,1)];
            xtrain1 =  [xtrain1; seq.var.Pi.data(i,1)];               % pitch
            %xtrain2 =  [xtrain2; smooth_pi];  % smooth Pitch
            xtrain3 =  [xtrain3;seq.var.P.data(i,1)];
            xtrain4 =  [xtrain4;seq.var.O.data(i,1)];          % ODBA
            %xtrain5 =  [xtrain5;smooth_o];          % smooth ODBA
            xtrain6 =  [xtrain6;seq.var.B2.data(i,4)];    % p2p stroke      
            xtrain7 =  [xtrain7;seq.var.B2.data(i,3)];    % Stroke freq 
            %                     data1 =  [data1(:,1); seq.var.X.data(lim_inf:lim_sup)];
            %                     data2 =  [data2(:,1); seq.var.Y.data(lim_inf:lim_sup)];          % ODBA
            %                     data_ref1 =  [data_ref1(:,1); seq_ref.var.X.data(lim_inf:lim_sup)];
            %                     data_ref2 =  [data_ref2(:,1); seq_ref.var.Y.data(lim_inf:lim_sup)];          % ODBA
        end
        
        
    end
    
    
% xtrain2 =  smoothdata(xtrain1,'movmean',coeff_smooth);  % smooth Pitch
% xtrain5 =  smoothdata(xtrain4,'movmean',coeff_smooth);

%xtrain = [ xtrain1 , xtrain2 , xtrain3, xtrain4 , xtrain5, xtrain6 , xtrain7];
xtrain  = xtrain5;

end


end




