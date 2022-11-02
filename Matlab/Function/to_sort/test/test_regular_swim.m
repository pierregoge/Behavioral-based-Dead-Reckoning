load('seq_c.mat')

ax = seq_c(7,1).var.Ad.data(:,1);
ay = seq_c(7,1).var.Ad.data(:,2);
az = seq_c(7,1).var.Ad.data(:,3);

pitch = seq_c(7,1).var.Pi.data(:,1);
t_stroke = zero_cross(ax);
t_stroke_y = zero_cross(ay);
t_stroke_z = zero_cross(az);

[p2p_stroke,min_stroke,max_stroke] = stroke_p2p(ax,t_stroke);
[p2p_stroke_y] = stroke_p2p(ay,t_stroke);
[p2p_stroke_z] = stroke_p2p(az,t_stroke);

freq_stroke_r = zeros(length(ax),1);
p2p_stroke_r = zeros(length(ax),1);

freq_stroke_r_y = zeros(length(ax),1);
p2p_stroke_r_y = zeros(length(ax),1);

freq_stroke_r_z = zeros(length(ax),1);
p2p_stroke_r_z = zeros(length(ax),1);

min_stroke_r = zeros(length(ax),1);
max_stroke_r = zeros(length(ax),1);

for i = 2:length(t_stroke(:,1))
    
freq_stroke(i,1) = (t_stroke(i,1)-t_stroke(i-1,1))/10;

freq_stroke_r(t_stroke(i-1,1):t_stroke(i,1),1) = freq_stroke(i,1);
p2p_stroke_r(t_stroke(i-1,1):t_stroke(i,1),1) = p2p_stroke(i,1);

p2p_stroke_r_y(t_stroke(i-1,1):t_stroke(i,1),1) = p2p_stroke_y(i,1);

p2p_stroke_r_z(t_stroke(i-1,1):t_stroke(i,1),1) = p2p_stroke_z(i,1);

min_stroke_r(t_stroke(i-1,1):t_stroke(i,1),1) = min_stroke(i,1);
max_stroke_r(t_stroke(i-1,1):t_stroke(i,1),1) = max_stroke(i,1);
end

inc = 1;
for i = 1:length(ax)
    
    if inc < 42
        if i > seq_c(7,1).var.B.data(inc+1,2)
            inc = inc+1;
        end
    elseif inc == 42
        if i > seq_c(7,1).var.B.data(inc,3)
            inc = inc+1;
        end
    end
    
        if (i >= seq_c(7,1).var.B.data(inc,2)) &&( i < seq_c(7,1).var.B.data(inc,3))
            current_b = seq_c(7,1).var.B.data(inc,1);
            
            if current_b ~= 3 && current_b ~= 1
                ax(i,1) = NaN;
            end
        end

    
    if max_stroke_r(i,1) > 0.05
        ax(i,1) = NaN;
    end
     if max_stroke_r(i,1) < 0.02
        ax(i,1) = NaN;
     end
     
    if min_stroke_r(i,1) > -0.02
        ax(i,1) = NaN;
    end
     if min_stroke_r(i,1) < -0.05
        ax(i,1) = NaN;
     end
     
     if freq_stroke_r(i,1) > 4
         ax(i,1) = NaN;
     end
       if freq_stroke_r(i,1) < 1.8
         ax(i,1) = NaN;
     end
    
end

ax_buf = NaN(length(ax(:,1)),1);
cpt = 0;
inc = 1;
for i = 1:length(ax)
    
    cpt = cpt + 1 ;
    
    if isnan(ax(i,1))
        
        if cpt >= 100
        ax_buf(i-cpt:i-1,1) = ax(i-cpt:i-1,1);
        lim_reg(inc,1) = i-cpt+1;
        lim_reg(inc,2) = i-1;
        inc=inc+1;
        end
        cpt = 0;
    end
        
end


ax = ax_buf;

%is regular
p2p_stroke_r = p2p_stroke_r.*(ax./ax);
freq_stroke_r = freq_stroke_r.*(ax./ax);

p2p_stroke_r_y = p2p_stroke_r_y.*(ax./ax);
p2p_stroke_r_z = p2p_stroke_r_z.*(ax./ax);


pitch = pitch.*(ax./ax);
speed_ref_7 = seq_s_ref(7,1).var.S10.data.Var1(1:end-1,1).*(ax./ax);

xtrain = [p2p_stroke_r,freq_stroke_r,ax,pitch,p2p_stroke_r+p2p_stroke_r_z,p2p_stroke_r_z];

for i=1:length(lim_reg(:,1))
    m_xtrain(i,:) = mean(xtrain(lim_reg(i,1):lim_reg(i,2),:));
    m_speed_ref_7(i,1) = mean(speed_ref_7(lim_reg(i,1):lim_reg(i,2),1));
end



plot_var_etho(ax,seq_c(7,1).var.B.data,1,'Ax')
plot_var_etho_2([seq_c(1,1).var.Ad.data(:,1),ax],seq_c(1,1).var.Pi.data(:,1),seq_c(1,1).var.B.data,1,'Ax','Pitch')


nb_str= length(t_stroke(:,1));
avg_str = mean(p2p_stroke);