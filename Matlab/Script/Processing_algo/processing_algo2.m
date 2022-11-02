nb = 2;

seq_b = downs_seq(seq_buf,100);   %Downsample to 1hz
seq_b.name = 'algo2';
seq(nb,:) = seq_b;
clear seq_b

% Low_pass filter
[seq(nb,:).var.Ag] = mov_mean_filter(2,seq(nb,:).var.A);  % For 1hz sampling
[seq(nb,:).var.Mg] = mov_mean_filter(2,seq(nb,:).var.M);  % For 1hz sampling

% Compute variables
[seq(nb,:).var.Ad] = var_accd(seq(nb,:).var);
[seq(nb,:).var.O]  = var_odba(seq(nb,:).var);
[seq(nb,:).var.J]  = var_jerk(seq(nb,:).var);

[seq(nb,:).var.Pi,seq(nb,:).var.Ro,seq(nb,:).var.He] = var_saam(seq(nb,:).var);

% Correction pitch offset
seq(nb,:).var.Pi.data = seq(nb,:).var.Pi.data -(9/180*pi);

% Cut seq in dives delimited by surface
surf = find_dives(seq(nb,:).var.P,0.2);
dives = f_seq2dives(surf,seq(nb,:));

%Ethogram apply on dive
dives = var_etho_V2(dives);

% Speed estimation in function of the ethogram
v = 2; %Version of the ethogram
speed_method = 1;
[dives] = s_etho_dive2s(dives,v,speed_method);

% Merge dives to a global sequence
seq(nb,:) = merge_dives2seq(dives(:,1));

% TODO script to function / Clean + seperate pitch and regular + comment
test_etho_b2_1hz

% Speed estimation in function of the ethogram
v = 2; %Version of the ethogram
speed_method = 1;
seq(nb,1) = s_etho_dive2s_2(seq(nb,1),v,speed_method);

% Set speed to speed calucalte with OCDR during high pitch phases
for i=1:length(seq(nb,1).var.S.data(:,1))-1
    
    % Applied OCDR function if high pitch phase
    if ~isnan(speed_pitch(i,1))
    seq(nb,1).var.S.data(i,:) = abs(speed_pitch(i,1));
    else
    seq(nb,1).var.S.data(i,:) = seq(2,1).var.S.data(i,1);    
    end
    % Fixed speed to zero fro REST and GROUND for algo error when high pitch phase and static phase overlaps 
    if seq(nb,1).var.B2.data(i,1) == 2 || seq(nb,1).var.B2.data(i,1) == 6
    seq(nb,1).var.S.data(i,:) = 0;
    end    
end

% Smooth speed acceleration between static and dynamic phases. The same
% smoothin is applied for all the algorithms. 
seq(nb,1).var.S.data(:,2) = smoothdata(seq(nb,1).var.S.data(:,1),'movmean',15);%*0.9;

clear v nb surf 