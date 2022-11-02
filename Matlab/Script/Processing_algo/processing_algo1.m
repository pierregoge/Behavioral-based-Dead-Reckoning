nb = 1;

% Downsample 10Hz
seq(nb,:) = downs_seq(seq_buf,10);   %Downsample to 10hz
seq(nb,:).name = 'algo1';

[seq(nb,:).var.Ag] = mov_mean_filter(10,seq(nb,:).var.A);  % For 10hz sampling
[seq(nb,:).var.Mg] = mov_mean_filter(10,seq(nb,:).var.M);  % For 10hz sampling

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

% Ethogram apply on dive
%dives = var_etho_V1(dives);
dives = var_etho_V5(dives);

% Speed estimation in function of the ethogram
v = 2; % Version of the ethogram
speed_method = 1;
[dives] = s_etho_dive2s(dives,v,speed_method);

% Merge dives to a global sequence
seq(nb,:) = merge_dives2seq(dives(:,1));

% downsample 1Hz
seq(nb,:) = downs_seq(seq(nb,:),10);   %Downsample to 1hz

% TODO script to function / Clean + seperate pitch and regular + comment
test_etho_b2_1hz

% Variabla use for the regular swimming regression
clear A B x 
A(:,1) = seq(1,1).var.B2.data(:,4);
A(:,2) = seq(1,1).var.B2.data(:,3);

% Add NaN when not regular for the regression
% for i=1:length(A(:,1))
%     if A(i,1) == 0
%         A(i,1:2) = NaN;
%     end
% end

% Load regression function
load('speed_accx_1.mat')

% Speed regression 
for i=1:length(seq(1,1).var.S.data(:,1))
    if seq(1,1).var.B2.data(i,2) == 1
       trainedModel_raw_1.predictFcn([A(i,:), 0*A(i,:)]);    
    end
end

% Fixed speed to zero fro REST and GROUND for algo error when regular phase and static phase overlaps 
seq(1,1).var.S.data( seq(1,1).var.B2.data(:,1) == 2) = 0;
seq(1,1).var.S.data( seq(1,1).var.B2.data(:,1) == 6) = 0;

% Set speed to speed calucalte with OCDR during high pitch phases
for i=1:length(seq(1,1).var.S.data(:,1))-1
    if ~isnan(speed_pitch(i,1))
    seq(1,1).var.S.data(i,1) = abs(speed_pitch(i,1));
    else
    seq(1,1).var.S.data(i,1) = seq(1,1).var.S.data(i,1);    
    end
end

% Smooth speed to remove spike on regular acceleration and smooth
% speed acceleration between static and dynamic phases. The same
% smoothing is applied for all the algorithms
seq(1,1).var.S.data(:,1) = smoothdata(seq(1,1).var.S.data(:,1),'movmean',15);

clear v nb surf 