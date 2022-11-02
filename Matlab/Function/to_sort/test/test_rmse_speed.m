
speed = ones(length(pitch(:,1)),1)*0.39;

for i=1:length(speed(:,1))-10
    if ~isnan(speed_down(i,1))
    speed(i,1) =speed_down(i,1);
    end
    if ~isnan(speed_up(i,1))
    speed(i,1) =speed_up(i,1);
    end
    if i < 10240
        speed(i,1) = 0;
        s(i,1)=0;
    end
end

speed_ref = seq_s_ref(nb_seq,1).var.S10.data.Var1(1:end-1,1);
rmse_s = sqrt(sum((speed(1:end)-speed_ref(1:end)).^2)/length(speed_ref(1:end)))
rmse_s_2 = sqrt(sum((s(1:end)-speed_ref(1:end-10)).^2)/length(speed_ref(1:end-10)))
