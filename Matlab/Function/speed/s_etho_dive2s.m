function [dives_out] = s_etho_dive2s_2(dives,v,speed_method)

if speed_method == 2 || speed_method == 3
    load('V1_f.mat');
    load('V2_f.mat');
end

if isstruct(dives(1,1))
    dives_out = dives;
    if ~isfield(dives(1,1),'type')
        fprintf('t_dive2traj: sctructure type is not defined \n') ;
        return
    else
        if ~strcmp( dives(1,1).type, 'dive' )
            fprintf('t_dive2traj: Need dives structure as input \n') ;
            return
        end
    end
    if ~isfield(dives(1,1).var,'B')
        fprintf('t_dive2traj: Need behavior variable \n') ;
        return
    end
else
    fprintf('t_dive2traj: Need structure as input \n') ;
    return
end

nb_dives = length(dives(:,1)); % number of dive for the sequence

for i=1:nb_dives
    %for i=1:70
        
        nb_behavior = length(dives(i,1).var.B.data(:,1)); % Number of behavior of the sequence
        
        for j=1:nb_behavior
            
            c_behavior = dives(i,1).var.B.data(j,1);  % Current behavior
            lim_inf = dives(i,1).var.B.data(j,2);     % Limit inferior of the behavior
            lim_sup = dives(i,1).var.B.data(j,3);     % Limit superior of the behavior
            
            if v == 1 % if version of ethogramme is V1
                
                if c_behavior == 1  % SWIM behavion
                    
                    if speed_method == 1
                        speed(lim_inf:lim_sup,1) = s_fix(0.37);
                        
                    elseif speed_method == 2
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        [nb_str_z, avg_str_z] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,3));
                        
                        speed(lim_inf:lim_sup,1) = 1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x;
                        xtrain(:,1) =  mean(dives(i,1).var.Pi.data(lim_inf:lim_sup,1));               % Mean pitch
                        xtrain(:,2) =  dives(i,1).var.P.data(lim_sup) - dives(i,1).var.P.data(lim_inf);    % Difference of depth
                        xtrain(:,3) =  lim_sup-lim_inf;                                      % Length of behavior in second
                        xtrain(:,4) =  mean(dives(i,1).var.P.data(lim_inf:lim_sup,1));                % mean depth
                        xtrain(:,5) =  (xtrain(end,2)/(sin(xtrain(end,1)))/xtrain(end,3));
                        xtrain(:,6) =  nb_str_x;               %
                        xtrain(:,7) =  avg_str_x;               %
                        xtrain(:,8) =  nb_str_z;               %
                        xtrain(:,9) =  avg_str_z;              %
                        speed(lim_inf:lim_sup,1) = abs(trainedModel_V1_f_swim.predictFcn(xtrain));
                        clear xtrain
                        if speed(lim_inf+1) > 0.8
                            speed(lim_inf:lim_sup,1) = s_fix(0.37);
                        end
                        
                        speed(lim_inf:lim_sup,1) = s_fix(0.37);
                        
                    elseif speed_method == 3
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        speed(lim_inf:lim_sup,1) = (1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x)*1;
                        speed(lim_inf:lim_sup,1) = s_fix(0.37);
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        [nb_str_z, avg_str_z] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,3));
                        
                        speed(lim_inf:lim_sup,1) = 1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x;
                        xtrain(:,1) =  mean(dives(i,1).var.Pi.data(lim_inf:lim_sup,1));               % Mean pitch
                        xtrain(:,2) =  dives(i,1).var.P.data(lim_sup) - dives(i,1).var.P.data(lim_inf);    % Difference of depth
                        xtrain(:,3) =  lim_sup-lim_inf;                                      % Length of behavior in second
                        xtrain(:,4) =  mean(dives(i,1).var.P.data(lim_inf:lim_sup,1));                % mean depth
                        xtrain(:,5) =  (xtrain(end,2)/(sin(xtrain(end,1)))/xtrain(end,3));
                        xtrain(:,6) =  nb_str_x;               %
                        xtrain(:,7) =  avg_str_x;               %
                        xtrain(:,8) =  nb_str_z;               %
                        xtrain(:,9) =  avg_str_z;              %
                        speed(lim_inf:lim_sup,1) = abs(trainedModel_V1_f_swim.predictFcn(xtrain));
                        clear xtrain
                        if speed(lim_inf+1) > 0.8
                            speed(lim_inf:lim_sup,1) = s_fix(0.37);
                        end
                        
                        speed(lim_inf:lim_sup,1) = s_fix(0.37);
                    end
                    
                end
                
                if c_behavior == 2  % REST behavior
                    speed(lim_inf:lim_sup,1) = s_fix(0);
                end
                
                if c_behavior == 3  % UP behavior
                    
                    if speed_method == 1
                        speed(lim_inf:lim_sup,1) = s_fix(0.478);
                    elseif speed_method == 2
                        
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        [nb_str_z, avg_str_z] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,3));
                        
                        speed(lim_inf:lim_sup,1) = 1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x;
                        xtrain(:,1) =  mean(dives(i,1).var.Pi.data(lim_inf:lim_sup,1));               % Mean pitch
                        xtrain(:,2) =  dives(i,1).var.P.data(lim_sup) - dives(i,1).var.P.data(lim_inf);    % Difference of depth
                        xtrain(:,3) =  lim_sup-lim_inf;                                      % Length of behavior in second
                        xtrain(:,4) =  mean(dives(i,1).var.P.data(lim_inf:lim_sup,1));                % mean depth
                        xtrain(:,5) =  (xtrain(end,2)/(sin(xtrain(end,1)))/xtrain(end,3));
                        xtrain(:,6) =  nb_str_x;               %
                        xtrain(:,7) =  avg_str_x;               %
                        xtrain(:,8) =  nb_str_z;               %
                        xtrain(:,9) =  avg_str_z;              %
                        speed(lim_inf:lim_sup,1) = trainedModel_V1_f_up.predictFcn(xtrain);
                        clear xtrain
                        speed(lim_inf:lim_sup,1) = s_fix(0.478);
                        
                    elseif speed_method == 3
                        
                        pitch = (dives(i,1).var.Pi.data(lim_inf:lim_sup,1));         % Pitch for the limit of the behavior
                        depth = dives(i,1).var.P.data(lim_inf:lim_sup,1);          % Depth for the limit of the behavior
                        Ad = dives(i,1).var.Ad.data(lim_inf:lim_sup,1);
                        fs = dives(i,1).var.Pi.sampling_rate;
                        [~,speed(lim_inf:lim_sup,1)] = s_reg2su_2(pitch,depth,Ad,fs);     % Apply speed fonction for UP behavior
                        
                        if speed(lim_inf+1) > 1
                            speed(lim_inf:lim_sup,1) = s_fix(0.478);
                        end
                        %speed(lim_inf:lim_sup,1) = s_fix(0.478);
                        
                    end
                    
                    
                end
                
                if c_behavior == 4  % DOWN behavior
                    
                    if speed_method == 1
                        speed(lim_inf:lim_sup,1) = s_fix(0.408);
                    elseif speed_method == 2
                        
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        [nb_str_z, avg_str_z] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,3));
                        
                        speed(lim_inf:lim_sup,1) = 1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x;
                        xtrain(:,1) =  mean(dives(i,1).var.Pi.data(lim_inf:lim_sup,1));               % Mean pitch
                        xtrain(:,2) =  dives(i,1).var.P.data(lim_sup) - dives(i,1).var.P.data(lim_inf);    % Difference of depth
                        xtrain(:,3) =  lim_sup-lim_inf;                                      % Length of behavior in second
                        xtrain(:,4) =  mean(dives(i,1).var.P.data(lim_inf:lim_sup,1));                % mean depth
                        xtrain(:,5) =  (xtrain(end,2)/(sin(xtrain(end,1)))/xtrain(end,3));
                        xtrain(:,6) =  nb_str_x;               %
                        xtrain(:,7) =  avg_str_x;               %
                        xtrain(:,8) =  nb_str_z;               %
                        xtrain(:,9) =  avg_str_z;              %
                        speed(lim_inf:lim_sup,1) = trainedModel_V1_f_down.predictFcn(xtrain);
                        clear xtrain
                        speed(lim_inf:lim_sup,1) = s_fix(0.408);
                        
                    elseif speed_method == 3
                        
                        
                        
                        pitch = (dives(i,1).var.Pi.data(lim_inf:lim_sup,1));         % Pitch for the limit of the behavior
                        depth = dives(i,1).var.P.data(lim_inf:lim_sup,1);          % Depth for the limit of the behavior
                        fs = dives(i,1).var.Pi.sampling_rate;
                        [~,speed(lim_inf:lim_sup,1)] = s_reg2sd_2(pitch,depth,fs);     % Apply speed fonction for UP behavior
                        
                        if speed(lim_inf+1) > 1
                            speed(lim_inf:lim_sup,1) = s_fix(0.408);
                        end
                        %speed(lim_inf:lim_sup,1) = s_fix(0.408);
                    end
                    
                    
                end
                
                if c_behavior == 5  % SURFACE behavior
                    
                    if speed_method == 1
                        
                        speed(lim_inf:lim_sup,1) = s_fix(0.4);
                        
                    elseif speed_method == 2
                        
                        speed(lim_inf:lim_sup,1) = s_fix(0.5);
                        
                    elseif speed_method == 3
                        
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        speed(lim_inf:lim_sup,1) = 1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x;
                        pitch = (dives(i,1).var.Pi.data(lim_inf:lim_sup,1));         % Pitch for the limit of the behavior
                        depth = dives(i,1).var.P.data(lim_inf:lim_sup,1);          % Depth for the limit of the behavior
                        fs = dives(i,1).var.Pi.sampling_rate;
                        speed(lim_inf:lim_sup,1) = s_reg2sd(pitch,depth,fs);     % Apply speed fonction for UP behavior
                        speed(lim_inf:lim_sup,1) = s_fix(0.4);
                    end
                    
                    
                    
                end
                
                
            elseif v == 2
                
                if c_behavior == 1  % SWIM behavion
                    
                    if speed_method == 1
                        speed(lim_inf:lim_sup,1) = s_fix(0.37);
                        
                        
                    elseif speed_method == 2
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        [nb_str_z, avg_str_z] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,3));
                        
                        speed(lim_inf:lim_sup,1) = 1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x;
                        xtrain(:,1) =  mean(dives(i,1).var.Pi.data(lim_inf:lim_sup,1));               % Mean pitch
                        xtrain(:,2) =  dives(i,1).var.P.data(lim_sup) - dives(i,1).var.P.data(lim_inf);    % Difference of depth
                        xtrain(:,3) =  lim_sup-lim_inf;                                      % Length of behavior in second
                        xtrain(:,4) =  mean(dives(i,1).var.P.data(lim_inf:lim_sup,1));                % mean depth
                        xtrain(:,5) =  (xtrain(end,2)/(sin(xtrain(end,1)))/xtrain(end,3));
                        xtrain(:,6) =  nb_str_x;               %
                        xtrain(:,7) =  avg_str_x;               %
                        xtrain(:,8) =  nb_str_z;               %
                        xtrain(:,9) =  avg_str_z;              %
                        speed(lim_inf:lim_sup,1) = abs(trainedModel_V1_f_swim.predictFcn(xtrain));
                        clear xtrain
                        if speed(lim_inf+1) > 0.8
                            speed(lim_inf:lim_sup,1) = s_fix(0.37);
                        end
                        
                        speed(lim_inf:lim_sup,1) = s_fix(0.30);
                        
                    elseif speed_method == 3
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        speed(lim_inf:lim_sup,1) = (1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x)*1;
                        %speed(lim_inf:lim_sup,1) = s_fix(0.37);
                        %[nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        %[nb_str_z, avg_str_z] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,3));
%                         
%                         speed(lim_inf:lim_sup,1) = 1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x;
%                         xtrain(:,1) =  mean(dives(i,1).var.Pi.data(lim_inf:lim_sup,1));               % Mean pitch
%                         xtrain(:,2) =  dives(i,1).var.P.data(lim_sup) - dives(i,1).var.P.data(lim_inf);    % Difference of depth
%                         xtrain(:,3) =  lim_sup-lim_inf;                                      % Length of behavior in second
%                         xtrain(:,4) =  mean(dives(i,1).var.P.data(lim_inf:lim_sup,1));                % mean depth
%                         xtrain(:,5) =  (xtrain(end,2)/(sin(xtrain(end,1)))/xtrain(end,3));
%                         xtrain(:,6) =  nb_str_x;               %
%                         xtrain(:,7) =  avg_str_x;               %
%                         xtrain(:,8) =  nb_str_z;               %
%                         xtrain(:,9) =  avg_str_z;              %
%                         speed(lim_inf:lim_sup,1) = abs(trainedModel_V1_f_swim.predictFcn(xtrain));
%                         clear xtrain
%                         if speed(lim_inf+1) > 0.8
%                             speed(lim_inf:lim_sup,1) = s_fix(0.37);
%                         end
                        
                        %speed(lim_inf:lim_sup,1) = s_fix(0.37);
                    end
                    
                end
                
                if c_behavior == 2  % REST behavior
                    speed(lim_inf:lim_sup,1) = s_fix(0);
                end
                
                if c_behavior == 3  % UP behavior
                    
                    if speed_method == 1
                        speed(lim_inf:lim_sup,1) = s_fix(0.478);
                    elseif speed_method == 3
                        
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        [nb_str_z, avg_str_z] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,3));
                        
                        speed(lim_inf:lim_sup,1) = 1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x;
                        xtrain(:,1) =  mean(dives(i,1).var.Pi.data(lim_inf:lim_sup,1));               % Mean pitch
                        xtrain(:,2) =  dives(i,1).var.P.data(lim_sup) - dives(i,1).var.P.data(lim_inf);    % Difference of depth
                        xtrain(:,3) =  lim_sup-lim_inf;                                      % Length of behavior in second
                        xtrain(:,4) =  mean(dives(i,1).var.P.data(lim_inf:lim_sup,1));                % mean depth
                        xtrain(:,5) =  (xtrain(end,2)/(sin(xtrain(end,1)))/xtrain(end,3));
                        xtrain(:,6) =  nb_str_x;               %
                        xtrain(:,7) =  avg_str_x;               %
                        xtrain(:,8) =  nb_str_z;               %
                        xtrain(:,9) =  avg_str_z;              %
                        speed(lim_inf:lim_sup,1) = trainedModel_V2_f_up.predictFcn(xtrain);
                        clear xtrain
                        speed(lim_inf:lim_sup,1) = s_fix(0.478);
                        
                    elseif speed_method == 3
                        
                        pitch = (dives(i,1).var.Pi.data(lim_inf:lim_sup,1));         % Pitch for the limit of the behavior
                        depth = dives(i,1).var.P.data(lim_inf:lim_sup,1);          % Depth for the limit of the behavior
                        Ad = dives(i,1).var.Ad.data(lim_inf:lim_sup,1);
                        fs = dives(i,1).var.Pi.sampling_rate;
                        [~,speed(lim_inf:lim_sup,1)] = s_reg2su(pitch,depth,Ad,fs);     % Apply speed fonction for UP behavior
                        
                        if speed(lim_inf+1) > 1
                            speed(lim_inf:lim_sup,1) = s_fix(0.478);
                        end
                        speed(lim_inf:lim_sup,1) = s_fix(0.478);
                        
                    end
                    
                    
                end
                
                if c_behavior == 4  % DOWN behavior
                    
                    if speed_method == 1
                        speed(lim_inf:lim_sup,1) = s_fix(0.408);
                    elseif speed_method == 2
                        
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        [nb_str_z, avg_str_z] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,3));
                        
                        speed(lim_inf:lim_sup,1) = 1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x;
                        xtrain(:,1) =  mean(dives(i,1).var.Pi.data(lim_inf:lim_sup,1));               % Mean pitch
                        xtrain(:,2) =  dives(i,1).var.P.data(lim_sup) - dives(i,1).var.P.data(lim_inf);    % Difference of depth
                        xtrain(:,3) =  lim_sup-lim_inf;                                      % Length of behavior in second
                        xtrain(:,4) =  mean(dives(i,1).var.P.data(lim_inf:lim_sup,1));                % mean depth
                        xtrain(:,5) =  (xtrain(end,2)/(sin(xtrain(end,1)))/xtrain(end,3));
                        xtrain(:,6) =  nb_str_x;               %
                        xtrain(:,7) =  avg_str_x;               %
                        xtrain(:,8) =  nb_str_z;               %
                        xtrain(:,9) =  avg_str_z;              %
                        speed(lim_inf:lim_sup,1) = trainedModel_V1_f_down.predictFcn(xtrain);
                        clear xtrain
                        speed(lim_inf:lim_sup,1) = s_fix(0.408);
                        
                    elseif speed_method == 3
                        
                        
                        
                        pitch = (dives(i,1).var.Pi.data(lim_inf:lim_sup,1));         % Pitch for the limit of the behavior
                        depth = dives(i,1).var.P.data(lim_inf:lim_sup,1);          % Depth for the limit of the behavior
                        fs = dives(i,1).var.Pi.sampling_rate;
                        [~,speed(lim_inf:lim_sup,1)] = s_reg2sd(pitch,depth,fs);     % Apply speed fonction for UP behavior
                        
%                         if lim_inf == 13
%                             a= 1;
%                         end
                        if speed(lim_inf) > 1
                            speed(lim_inf:lim_sup,1) = s_fix(0.37);
                        end
                        speed(lim_inf:lim_sup,1) = s_fix(0.408);
                    end
                    
                    
                end
                
                if c_behavior == 5  % SURFACE behavior
                    
                    if speed_method == 1
                        
                        speed(lim_inf:lim_sup,1) = s_fix(0);
                        
                    elseif speed_method == 2
                        
                        speed(lim_inf:lim_sup,1) = s_fix(0);
                        
                    elseif speed_method == 3
                        
                        [nb_str_x, avg_str_x] = stroke_nb(dives(i,1).var.Ad.data(lim_inf:lim_sup,1));
                        speed(lim_inf:lim_sup,1) = 1*0.0437+nb_str_x*-0.0017+5.5431*avg_str_x;
                        pitch = (dives(i,1).var.Pi.data(lim_inf:lim_sup,1));         % Pitch for the limit of the behavior
                        depth = dives(i,1).var.P.data(lim_inf:lim_sup,1);          % Depth for the limit of the behavior
                        fs = dives(i,1).var.Pi.sampling_rate;
                        speed(lim_inf:lim_sup,1) = s_reg2sd(pitch,depth,fs);     % Apply speed fonction for UP behavior
                        speed(lim_inf:lim_sup,1) = s_fix(0);
                    end
                    
                    
                    
                end
                
                if c_behavior == 6  % Ground behavior
                    
                    if speed_method == 1
                        
                        speed(lim_inf:lim_sup,1) = s_fix(0);
                        
                    elseif speed_method == 2
                        
                        speed(lim_inf:lim_sup,1) = s_fix(0);
                        
                    elseif speed_method == 3
                        
                        
                        speed(lim_inf:lim_sup,1) = s_fix(0);
                    end
                    
                    
                    
                end
                
                
                
            end
            
            
        end
        
        dives_out(i,1).var.S.data = speed;
        dives_out(i,1).var.S.sampling = dives(i,1).var.A.sampling;
        dives_out(i,1).var.S.sampling_rate = dives(i,1).var.A.sampling_rate;
        dives_out(i,1).var.S.sampling_rate_unit = dives(i,1).var.A.sampling_rate_unit;
        dives_out(i,1).var.S.name = 'S';
        dives_out(i,1).var.S.full_name = 'Speed';
        
        clear speed
        
    end
    
%end

