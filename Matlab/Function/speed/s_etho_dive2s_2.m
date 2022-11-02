function [dives_out] = s_etho_dive2s_2(dives,v,speed_method)

    dives_out = dives;
    length_dive = length(dives(1,1).var.B2.data(:,1)); % Number of behavior of the sequence
    
    for j=1:length_dive
        
        c_behavior_1 = dives(1,1).var.B2.data(j,1);  % Current behavior
        c_behavior_2 = dives(1,1).var.B2.data(j,2);  % Current behavior
        
        if v == 2
            
            if c_behavior_1 == 1 && c_behavior_2 == 1 % SWIM behavion
                
                if speed_method == 1
                    speed(j,1) = s_fix(0.3697);
                end
            end
            
            if c_behavior_1 == 1 && c_behavior_2 == 0 % SWIM behavion
                
                if speed_method == 1
                    speed(j,1) = s_fix(0.3502);
                end
            end
            
            
            if c_behavior_1 == 2  % REST behavior
                speed(j,1) = s_fix(0);
            end
            
            if c_behavior_1 == 3 && c_behavior_2 == 1 % UP behavior
                
                if speed_method == 1
                    speed(j,1) = s_fix(0.4683);
                end
            end
            
             if c_behavior_1 == 3 && c_behavior_2 == 0 % UP behavior
                
                if speed_method == 1
                    speed(j,1) = s_fix(0.4725);
                end
            end
            
            if c_behavior_1 == 4 && c_behavior_2 == 1 % DOWN behavior
                
                if speed_method == 1
                    speed(j,1) = s_fix(0.408);
                end
            end
            
              if c_behavior_1 == 4 && c_behavior_2 == 0 % DOWN behavior
                
                if speed_method == 1
                    speed(j,1) = s_fix(0.3916);
                end
            end
            
            if c_behavior_1 == 5  % SURFACE behavior
                
                if speed_method == 1
                    speed(j,1) = s_fix(0.3648);
                end
                
            end
            
            if c_behavior_1 == 6  % Ground behavior
                
                if speed_method == 1
                    speed(j,1) = s_fix(0.0);
                end
            end
            
            
            
        end
        
        
    end
    
    dives_out(1,1).var.S.data = speed;
    dives_out(1,1).var.S.sampling = dives(1,1).var.A.sampling;
    dives_out(1,1).var.S.sampling_rate = dives(1,1).var.A.sampling_rate;
    dives_out(1,1).var.S.sampling_rate_unit = dives(1,1).var.A.sampling_rate_unit;
    dives_out(1,1).var.S.name = 'S';
    dives_out(1,1).var.S.full_name = 'Speed';
    
    clear speed
    
end



