function [analyse_up_speed] = speed_pitch_depth_2(analyse_up_speed)


for i=2:length(analyse_up_speed(:,1))
    diff_x_ref_buf(i,1) = analyse_up_speed(i,1)-analyse_up_speed(i-1,1);
    diff_y_ref_buf(i,1) = analyse_up_speed(i,2)-analyse_up_speed(i-1,2);
end

mean_pitch_buf = mean(analyse_up_speed(:,12));
diff_depth_buf = analyse_up_speed(end,14)-analyse_up_speed(1,14);

analyse_up_speed(:,4) = sum(sqrt(diff_x_ref_buf(:,1).^2+diff_y_ref_buf(:,1).^2));

analyse_up_speed(:,17) = -1*(diff_depth_buf/(tan(mean_pitch_buf/180*pi)));
analyse_up_speed(:,18) = mean_pitch_buf;
analyse_up_speed(:,19) = length(analyse_up_speed(:,1));
analyse_up_speed(:,20) = mean(analyse_up_speed(:,14));

end

