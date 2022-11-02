load('seq_c.mat')
clear lim_reg m_speed m_speed_ref_7 m_xtrain m_pitch
nb_seq = 7;

ax = seq_c(nb_seq,1).var.Ad.data(:,1);
pitch = seq_c(nb_seq,1).var.Pi.data(:,1)+5/180*pi;
depth = seq_c(nb_seq,1).var.P.data(:,1);
pitch_f = NaN(length(pitch(:,1)),1);

for i = 1:length(pitch)
    
    if pitch(i,1)*180/pi > -20
        pitch_f(i,1) = NaN;
    else
        pitch_f(i,1) = pitch(i,1);
    end

end

pitch_buf = NaN(length(pitch(:,1)),1);
cpt = 0;
inc = 1;
for i = 1:length(pitch)
    
    cpt = cpt + 1 ;
    
    if isnan(pitch_f(i,1))
        
        if cpt >= 80
        pitch_buf(i-cpt:i-1,1) = pitch(i-cpt:i-1,1);
        lim_reg(inc,1) = i-cpt+1;
        lim_reg(inc,2) = i-1;
        inc=inc+1;
        end
        cpt = 0;
    end
        
end


pitch = pitch_buf;
speed_ref = seq_s_ref(nb_seq,1).var.S10.data.Var1(1:end-1,1).*(pitch./pitch);

speed_down = NaN(length(pitch(:,1)),1);

for i=1:length(lim_reg(:,1))
    m_pitch(i,1) = mean(pitch(lim_reg(i,1):lim_reg(i,2),1));
    diff_depth(i,1) = depth(lim_reg(i,2),1)-depth(lim_reg(i,1),1);
    m_speed(i,1) = abs((abs(diff_depth(i,1))./tan(m_pitch(i,1)))/((lim_reg(i,2)-lim_reg(i,1))/10));
    m_speed_ref_7(i,1) = mean(speed_ref(lim_reg(i,1):lim_reg(i,2),1));
    speed_down(lim_reg(i,1):lim_reg(i,2),1) = m_speed(i,1);

end

m_xtrain = [m_speed,m_pitch];

rmse_s = sqrt(sum((m_speed-m_speed_ref_7).^2)/length(m_speed(:,1)))

len_s= 0;
for i = 1:length(speed_down)-1

       if ~isnan(speed_down(i,1))
       len_s = len_s+1;
       
       diff_depth(i,1) = depth(i+1,1)-depth(i,1);
       speed_down(i,2) = abs((abs(diff_depth(i,1))./tan(pitch(i,1)))*10);
       
       buf_diff_pow_down(len_s,1)=(speed_down(i,1)-speed_ref(i,1)).^2;
       buf_diff_pow_down(len_s,2)=(speed_down(i,2)-speed_ref(i,1)).^2;
       %buf_diff_pow_down(len_s,3)=(s(i,1)-speed_ref(i,1)).^2;
       

    
       end
end

rmse_s = sqrt(sum((buf_diff_pow_down(:,1)))/len_s)
rmse_s_2 = sqrt(sum((buf_diff_pow_down(:,2)))/len_s)
rmse_s_3 = sqrt(sum((buf_diff_pow_down(:,3)))/len_s)


plot_var_etho(pitch_buf,seq_c(nb_seq,1).var.B.data,1,'Ax')

%plot_var_etho_2([seq_c(nb_seq,1).var.Ad.data(:,1),ax],seq_c(nb_seq,1).var.Pi.data(:,1),seq_c(nb_seq,1).var.B.data,1,'Ax','Pitch')


