function [lim,speed_pitch,etho_pitch,etho_regular,etho_regular_mean,lim_reg] = etho_pitch_reg_1(seq)

pitch = seq.var.Pi.data(:,1);
depth = seq.var.P.data(:,1);

len_dive = length(seq.var.Pi.data(:,1));

%% Etho pitch + regular

cpt_d = 0;
cpt_u = 0;
inc = 1;

for j=2:len_dive
    if pitch(j,1)*180/pi > -20 && pitch(j,1)*180/pi < 0    % Pitch sup down lim
        pitch_bev(j,1) = NaN;
    elseif pitch(j,1)*180/pi < 17 && pitch(j,1)*180/pi > 0  % Pitch inf up lim
        pitch_bev(j,1) = NaN;
    elseif pitch(j,1)*180/pi > 17 && pitch(j,1)*180/pi > 0  % Pitch up
        pitch_bev(j,1) = 4;
        cpt_u = cpt_u + 1 ;
    elseif pitch(j,1)*180/pi < -20 && pitch(j,1)*180/pi < 0  % Pitch Down
        pitch_bev(j,1) = 3;
        cpt_d = cpt_d + 1 ;
    end
    
    
    if pitch_bev(j,1) ~= 3
        if cpt_d >= 8 %40 % old 8
            lim(inc,1) = 4;
            lim(inc,2) = j-cpt_d+1;
            lim(inc,3) = j-1;
            inc=inc+1;
        end
        cpt_d = 0;
    end
    if pitch_bev(j,1) ~= 4
        if cpt_u >= 10 %50  %old 10
            lim(inc,1) = 3;
            lim(inc,2) = j-cpt_u+1;
            lim(inc,3) = j-1;
            inc=inc+1;
        end
        cpt_u = 0;
    end
end

etho_pitch = NaN(length(pitch(:,1)),1);
speed_pitch = NaN(length(pitch(:,1)),1);

for i=1:length(lim(:,1))
    m_pitch(i,1) = mean(pitch(lim(i,2):lim(i,3),1));
    diff_depth(i,1) = depth(lim(i,3),1)-depth(lim(i,2),1);
    lim(i,4) = (abs(diff_depth(i,1))./tan(m_pitch(i,1)))/((lim(i,3)-lim(i,2)));
    %m_speed_ref_7(i,1) = mean(speed_ref(lim(i,1):lim(i,2),1));
    speed_pitch(lim(i,2):lim(i,3),1) = lim(i,4);
    etho_pitch(lim(i,2):lim(i,3),1) =  lim(i,1);
end



ax = seq.var.Ad.data(:,1);

t_stroke = zero_cross(ax);
[p2p_stroke,min_stroke,max_stroke] = stroke_p2p(ax,t_stroke);

freq_stroke_r = zeros(length(ax),1);
p2p_stroke_r = zeros(length(ax),1);
min_stroke_r = zeros(length(ax),1);
max_stroke_r = zeros(length(ax),1);

for i = 2:length(t_stroke(:,1))
    
freq_stroke(i,1) = (t_stroke(i,1)-t_stroke(i-1,1))/10;

freq_stroke_r(t_stroke(i-1,1):t_stroke(i,1),1) = freq_stroke(i,1);
p2p_stroke_r(t_stroke(i-1,1):t_stroke(i,1),1) = p2p_stroke(i,1);
min_stroke_r(t_stroke(i-1,1):t_stroke(i,1),1) = min_stroke(i,1);
max_stroke_r(t_stroke(i-1,1):t_stroke(i,1),1) = max_stroke(i,1);

end

inc = 1;
for i = 1:length(ax)
    
    if max_stroke_r(i,1) > 0.05
        ax(i,1) = NaN;
    end
    if max_stroke_r(i,1) < 0.005  % For 1Hz
        ax(i,1) = NaN;
    end
    
    if min_stroke_r(i,1) > -0.005
        ax(i,1) = NaN;
    end
    if min_stroke_r(i,1) < -0.05
        ax(i,1) = NaN;
    end
    
%     if freq_stroke_r(i,1) > 4
%         ax(i,1) = NaN;
%     end
%     if freq_stroke_r(i,1) < 1.8
%         ax(i,1) = NaN;
%     end
    
end

etho_regular = NaN(length(ax(:,1)),4);
cpt = 0;
inc = 1;
for i = 1:length(ax)
    
    cpt = cpt + 1 ;
    
    if isnan(ax(i,1)) 
        if cpt >= 5% 1sec
            etho_regular(i-cpt:i-1,1) = freq_stroke_r(i-cpt:i-1,1);
            etho_regular(i-cpt:i-1,2) = p2p_stroke_r(i-cpt:i-1,1);
            etho_regular(i-cpt:i-1,3) = min_stroke_r(i-cpt:i-1,1);
            etho_regular(i-cpt:i-1,4) = max_stroke_r(i-cpt:i-1,1);
            lim_reg(inc,1) = i-cpt+1;
            lim_reg(inc,2) = i-1;
            inc=inc+1;
        end
        cpt = 0;
    end
        
end

for i=1:length(lim_reg(:,1))
    for j=1:4
    etho_regular_mean(lim_reg(i,1):lim_reg(i,2),j) = mean(etho_regular(lim_reg(i,1):lim_reg(i,2),j));
    end
end


end

