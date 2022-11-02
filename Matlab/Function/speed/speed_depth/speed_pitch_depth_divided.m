function [analyse_down_speed] = speed_pitch_depth_divided(analyse_down_speed,nb_div)


j=1;
inc=1;

for i=1:length(analyse_down_speed(:,1))
    
    if analyse_down_speed(i,16) == inc+1
        
        rest = rem(i-j,nb_div);% Rest part
        int = fix((i-j)/nb_div); % interger part
        
        for ii=1:nb_div-1
            
            inc1 = j;
            
            for jj=inc1+1:inc1+int
                diff_x_down_buf(jj,1) = analyse_down_speed(jj,1)-analyse_down_speed(jj-1,1);
                diff_y_down_buf(jj,1) = analyse_down_speed(jj,2)-analyse_down_speed(jj-1,2);
            end
            
            if ii == 1
                diff_x_down_buf(inc1+1,1) = diff_x_down_buf(inc1+2,1);
                diff_y_down_buf(inc1+1,1) = diff_y_down_buf(inc1+2,1);
            end
            
            mean_pitch_buf = mean(analyse_down_speed(inc1+1:inc1+int,12));
            
            diff_depth_buf = analyse_down_speed(inc1+int,14)-analyse_down_speed(inc1+1,14);
            
            analyse_down_speed(inc1+1:inc1+int,4) = sum(sqrt(diff_x_down_buf(inc1+1:inc1+int,1).^2+diff_y_down_buf(inc1+1:inc1+int,1).^2));
            
            analyse_down_speed(inc1+1:inc1+int,17) = -1*(diff_depth_buf/(tan(mean_pitch_buf/180*pi)));
            analyse_down_speed(inc1+1:inc1+int,18) = mean_pitch_buf;
            analyse_down_speed(inc1+1:inc1+int,19) = (inc1+int)-(inc1+1);
            analyse_down_speed(inc1+1:inc1+int,20) = mean(analyse_down_speed(inc1+1:inc1+int,14));
            
            
            j=jj;
      
        end
        
        inc1 = j;
        
        for jj=inc1+1:inc1+int+rest-1
            diff_x_down_buf(jj,1) = analyse_down_speed(jj,1)-analyse_down_speed(jj-1,1);
            diff_y_down_buf(jj,1) = analyse_down_speed(jj,2)-analyse_down_speed(jj-1,2);
        end
        
        mean_pitch_buf = mean(analyse_down_speed(inc1+1:inc1+int+rest-1,12));
        
        diff_depth_buf = analyse_down_speed(inc1+int+rest-1,14)-analyse_down_speed(inc1+1,14);
        
        analyse_down_speed(inc1+1:inc1+int+rest-1,4) = sum(sqrt(diff_x_down_buf(inc1+1:inc1+int+rest-1,1).^2+diff_y_down_buf(inc1+1:inc1+int+rest-1,1).^2));
        
        analyse_down_speed(inc1+1:inc1+int+rest-1,17) = -1*(diff_depth_buf/(tan(mean_pitch_buf/180*pi)));
        analyse_down_speed(inc1+1:inc1+int+rest-1,18) = mean_pitch_buf;
        analyse_down_speed(inc1+1:inc1+int+rest-1,19) = (inc1+int+rest-1)-(inc1+1);
        analyse_down_speed(inc1+1:inc1+int+rest-1,20) = mean(analyse_down_speed(inc1+1:inc1+int+rest-1,14));
        
        
        j=i-1;
        
        inc = inc+1;
        
    end
end

