%% S10E - Non adaptative method at 10Hz. 
% Method number 4 in the scrpit/ 

nb = 4;
load('speed_odba_v1.mat')

% downsample 5Hz
seq_b = downs_seq(seq_buf,10);   %Downsample to 10hz
seq_b.name = 'algo4';
seq(nb,:) = seq_b;
clear seq_b

% Low_pass filter
[seq(nb,:).var.Ag] = mov_mean_filter(10,seq(nb,:).var.A);  % For 1hz sampling
[seq(nb,:).var.Mg] = mov_mean_filter(10,seq(nb,:).var.M);  % For 1hz sampling
% [seq.var.Ag] = mov_mean_filter(4,seq.var.A);  % For 1hz sampling
% [seq.var.Mg] = mov_mean_filter(4,seq.var.M);  % For 1hz sampling

% Compute variables
[seq(nb,:).var.Ad] = var_accd(seq(nb,:).var);
[seq(nb,:).var.O]  = var_odba(seq(nb,:).var);
[seq(nb,:).var.J]  = var_jerk(seq(nb,:).var);

[seq(nb,:).var.Pi,seq(nb,:).var.Ro,seq(nb,:).var.He] = var_saam(seq(nb,:).var);

% Correction pitch offset
seq(nb,:).var.Pi.data = seq(nb,:).var.Pi.data -(9/180*pi);

% Find out Jerk. Need to be done at 5hz minimum
% seq(nb,:).var.J = seq(nb,:).var.A;
% seq(nb,:).var.J.data = njerk(seq(nb,:).var.A);



% Cut seq in dives delimited by surface
surf = find_dives(seq(nb,:).var.P,0.2);
dives = f_seq2dives(surf,seq(nb,:));

%Ethogram apply on dive
dives = var_etho_V3(dives);

% Speed estimation in function of the ethogram
v = 1; %Version of the ethogram
speed_method = 2;
[dives] = s_etho_dive2s(dives,v,speed_method);

% % Merge dives to a global sequence
seq(nb,:) = merge_dives2seq(dives(:,1));

%downsample 1Hz
seq(nb,:) = downs_seq(seq(nb,:),10);   %Downsample to 1hz
 
smooth_odba = smoothdata(seq(nb,:).var.O.data,'movmean',5);

seq(nb,1).var.S.data =  trainedModel_odba.predictFcn(smooth_odba);

nb_behavior = length(seq(nb,1).var.B.data(:,1)); % Number of behavior of the sequence


%Setting the speed to 0 during resting phase even for no adaptative method
for j=1:nb_behavior
    
    c_behavior = seq(nb,1).var.B.data(j,1);  % Current behavior
    lim_inf = seq(nb,1).var.B.data(j,2);     % Limit inferior of the behavior
    lim_sup = seq(nb,1).var.B.data(j,3);     % Limit superior of the behavior
    
    if c_behavior == 2  % rest behavion
        
         seq(nb,1).var.S.data(lim_inf:lim_sup,1) = s_fix(0);
           
    end
end

 
seq(nb,1).var.S.data(:,1) = smoothdata(seq(nb,1).var.S.data(:,1),'movmean',15);
seq(nb,1).var.S.sampling = seq(nb,1).var.A.sampling;
seq(nb,1).var.S.sampling_rate = seq(nb,1).var.A.sampling_rate;
seq(nb,1).var.S.sampling_rate_unit = seq(nb,1).var.A.sampling_rate_unit;
seq(nb,1).var.S.name = 'S';
seq(nb,1).var.S.full_name = 'Speed';

clear v nb surf 