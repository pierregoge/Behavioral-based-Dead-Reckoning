[lim,speed_pitch,etho_pitch,etho_regular,lim_reg,etho_regular_mean] = etho_pitch_reg(seq(1,1));

seq(1,1).var.B2 = seq(1,1).var.A(:,1);
seq(1,1).var.B2.data = seq(1,1).var.A.data(:,1)*0;

inc = 1;
for i=1:length(etho_regular(:,1))-2
    
    if ~isnan(etho_regular(i,1))
        buf(inc,1) = etho_regular(i,1);
        %buf(inc,2) = seq_s_ref(7,1).var.S.data.Var1(i,1);
        inc = inc+1;
        seq(1,1).var.B2.data(i,2) = 1;
        seq(1,1).var.B2.data(i,3:6) = etho_regular(i,:);
    end
end

for i=1:length(etho_pitch(:,1))-2
    if ~isnan(etho_pitch(i,1))
        seq(1,1).var.B2.data(i,2) = etho_pitch(i,1);
    end
end


for i=1:length(seq(1,1).var.B.data(:,1))    
            bev_buf = seq(1,1).var.B.data(i,1) ;
            seq(1,1).var.B2.data(seq(1,1).var.B.data(i,2):seq(1,1).var.B.data(i,3),1) = bev_buf;
end

