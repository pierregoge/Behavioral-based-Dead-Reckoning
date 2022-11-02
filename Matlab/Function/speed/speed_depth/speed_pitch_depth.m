function [analyse_up_speed] = speed_pitch_depth(analyse_up_speed)

j=1;
inc=1;
for i=1:length(analyse_up_speed(:,1))
    
    if analyse_up_speed(i,16) == inc+1
    
    for jj=j+1:i-1
    diff_x_ref_buf(jj,1) = analyse_up_speed(jj,1)-analyse_up_speed(jj-1,1);
    diff_y_ref_buf(jj,1) = analyse_up_speed(jj,2)-analyse_up_speed(jj-1,2);
    end
    
    mean_pitch_buf = mean(analyse_up_speed(j:i-1,12));
    diff_depth_buf = analyse_up_speed(i-1,14)-analyse_up_speed(j,14);
    
    analyse_up_speed(j:i-1,4) = sum(sqrt(diff_x_ref_buf(j:i-1,1).^2+diff_y_ref_buf(j:i-1,1).^2));
    
    analyse_up_speed(j:i-1,17) = -1*(diff_depth_buf/(tan(mean_pitch_buf/180*pi)));
    analyse_up_speed(j:i-1,18) = mean_pitch_buf;
    analyse_up_speed(j:i-1,19) = i-j;
    analyse_up_speed(j:i-1,20) = mean(analyse_up_speed(j:i-1,14));
    
    j=i;
    inc = inc+1; 
    
    end
           
end


end

