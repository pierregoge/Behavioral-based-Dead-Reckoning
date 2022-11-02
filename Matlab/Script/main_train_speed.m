% Script to format sensor data
clear all

% Import tag data in timetables
main_import_tag_data

%Script for reference data
main_ref_seq

%Script for reference data
main_ref_seq


%% format data tag
[var] = f_tt2var(data_tag,100);
[seq_b] = f_var2seq(var);

%% Format data ref
len = length(data_ref(:)); % Number of valid sequence for validation

[seq_c_ref] = f_tt2seq_ref(data_c_ref,1); %Store all the ref sequence
[seq_ref] = f_tt2seq_raw_ref(data_ref,1); %Store all the ref sequence
%var_ref.S = resample(x,ups,dns);

%% Compute var

%profile on
% seq.var.A.data = seq.var.A.data(1:20,1);
% seq.var.P.data = seq.var.P.data(1:20,1);
% [seq.var.Ag] = mov_mean_filter(7,seq.var.A);
% profileStruct = profile('info');
% [flopTotal,Details] = FLOPS('mov_mean_filter','exampleFunMat',profileStruct);

% seq.var.A.data = seq.var.A.data(1:100,1);
% seq.var.P.data = seq.var.P.data(1:100,1);
% profile on
% s=ocdr(seq.var.P.data(1:100,1),seq.var.A.data(1:100,1),5,0.25);
% profileStruct = profile('info');
% [flopTotal,Details] = FLOPS('ocdr','exampleFunMat',profileStruct);

nb = 1;

% downsample 5Hz
seq_b = downs_seq(seq_b,100);   %Downsample to 10hz
seq_b.name = 'algo2';
seq(nb,:) = seq_b;
clear seq_b

% Low_pass filter
[seq(nb,:).var.Ag] = mov_mean_filter(2,seq(nb,:).var.A);  % For 1hz sampling
[seq(nb,:).var.Mg] = mov_mean_filter(2,seq(nb,:).var.M);  % For 1hz sampling
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

% downsample 1Hz
%seq(nb,:) = downs_seq(seq(nb,:),10);   %Downsample to 1hz

% Cut seq in dives delimited by surface
surf = find_dives(seq(nb,:).var.P,0.2);
dives = f_seq2dives(surf,seq(nb,:));

%Ethogram apply on dive
dives = var_etho_V2(dives);

% Speed estimation in function of the ethogram
v = 2; %Version of the ethogram
speed_method = 1;
[dives] = s_etho_dive2s(dives,v,speed_method);

% % Merge dives to a global sequence
seq(nb,:) = merge_dives2seq(dives(:,1));

%downsample 1Hz
%seq = downs_seq(seq,10);   %Downsample to 1hz

% Crop sequence with ref sequence limits
for i=1:length(seq_c_ref(:,1))
    lim_inf = seq_c_ref(i,1).tstart;
    lim_sup = seq_c_ref(i,1).tend-1;
    [seq_c(i,1)] = crop_seq(seq(nb,:),lim_inf,lim_sup);
end


% J = njerk(seq.var.A);
% plot_var_etho(J(1:end-1,1),seq.var.B.data(:,:),1,'Jerk')

%% Calculate trajectory
offset_h = -19.32;

[seq_track] = seq_seq2htrack(seq,offset_h);

[seq_c_track] = seq_seq2htrack(seq_c(7,1),offset_h);

%% Calculate trajectory seq

for i=1:7
    
    [seq_c_track(i,1)] = seq_seq2htrack(seq_c(i,1),offset_h);
    
end

plot_var_etho(seq_track.var.P.data(:,1),seq_track.var.B.data(:,:),1,'Depth')



%% TRAJ FROM MARK JOHNSON


seq.var.A.data(:,3) = seq.var.A.data(:,3);
seq.var.A.data(:,2) = -seq.var.A.data(:,2);
seq.var.A.data(:,1) = -seq.var.A.data(:,1);

% Vertical speed
[s_depth,v] = ocdr(seq.var.P.data(:,1),seq.var.A.data(:,1:3),1);
% Horizontal speed : Need mean speed. We find by analysing reference speed

od = odba(seq.var.A.data(:,1:3),1,0.25);
mean_s = 0.418;
%s = sqrt(max(mean_s.^2-v.^2,0));
% ODBA
if seq.var.A.sampling_rate == 1
    odba_threshold = 0.0056; % ODBA threshold for no moving phases
elseif seq.var.A.sampling_rate == 10
    odba_threshold = 0.0056; % ODBA threshold for no moving phases
end

s = zeros(length(s_depth(:,1)),1);
for i=1:length(s(:,1))
    
    %if s(i,1) == 0
    s(i,1) = mean_s;
    %end
    
    if ~isnan(s_depth(i,1))
        s(i,1) = abs(s_depth(i,1));
    end
    
    if od(i,1) < odba_threshold % We Initialy 0.0026
        s(i,1) = 0;
    end
    
    if s(i,1) > 1 % We Initialy 0.0026
        s(i,1) = 1;
    end
end

[T,pe] = htrack(seq.var.A.data(:,1:3),seq.var.M.data(:,1:3),s,1,0.25) ;
%[T1,pe] = htrack(seq_c.var.A.data(:,1:3),seq_c.var.M.data(:,1:3),s1,1,0.25) ;

% Crop sequence with ref sequence limits
s_c = [];
x_c = [];
y_c = [];

for i=1:length(seq_c_ref(:,1))
    lim_inf = seq_c_ref(i,1).tstart;
    lim_sup = seq_c_ref(i,1).tend-1;
    
    s_c = [s_c; s(lim_inf:lim_sup,1)-s(lim_inf,1)];
    x_c = [x_c; T(lim_inf:lim_sup,2)-T(lim_inf,2)];
    y_c = [y_c; T(lim_inf:lim_sup,1)-T(lim_inf,1)];
end

%% Plot traj

[seq_c_track] = crop_seq(seq_track,2500,10739);
[seq_c_r] = crop_seq(seq_ref,2500,10739);

buf = seq_c_r.var.X.data;
buf1 = seq_c_r.var.Y.data;
seq_c_r.var.X.data = -buf1;
seq_c_r.var.Y.data = -buf;

X(:,1) = seq_c_r.var.X.data;
X(:,2) = seq_c_track.var.X.data;
X(:,3) = T(2500:10739,2)-T(2500,2);
plot_var_etho(X,seq_c_track.var.B.data(:,:),1,'Displacement X')

plot_var_etho(seq_c_track.var.P.data(:,1),seq_c_track.var.B.data(:,:),1,'Depth')








% %plot(seq_track.var.X.data,seq_track.var.Y.data);
% plot(seq_c_track.var.X.data,seq_c_track.var.Y.data);
% axis equal;
% hold on;
% plot(seq_c_r.var.X.data,seq_c_r.var.Y.data);
%

% S(:,1) = seq_c_ref(7,1).var.S.data.Var1(1:end-1);
% S(:,2) = seq_c_track.var.S.data(:,1);
%Y(:,3) = T(1101:end,1)-T(1101,1);
%plot_var_etho(Y,seq_c.var.B.data(:,:),1,'Displacement Y')
% plot_var_etho((Y(:,1)-Y(:,2)),seq_c.var.B.data(:,:),1,'Diff Y')
% plot_var_etho((S(:,1)-S(:,2)),seq_c.var.B.data(:,:),1,'Diff S')
%
Y(:,1) = seq_c_r.var.Y.data;
Y(:,2) = seq_c_track.var.Y.data(:,1);
Y(:,3) = T(2500:10739,1)-T(2500,1);

% offset_y =0;
% for i=2:length(seq_c_track.var.S.data(:,1))
% if seq_c_track.var.S.data(i,1) ~= 0
% Y(i,2) = seq_c_track.var.Y.data(i,1)+0.05*i+offset_y;
% elseif seq_c_track.var.S.data(i,1) == 0
% Y(i,2) = Y(i-1,2);
% end

%if seq_c_track.var.S.data(i,1) ~= 0 && seq_c_track.var.S.data(i-1,1) == 0
%Y(i,2) = Y(i-1,2);
%seq_c_track.var.Y.data(:,1) = seq_c_track.var.Y.data(:,1) - abs((Y(i-1,2)-seq_c_track.var.Y.data(i,1)-0.05*i));
%offset_y = -1*abs((Y(i-1,2)-seq_c_track.var.Y.data(i,1)));
%end
%
%end

%
% Y(:,3) = T(2000:10739,1)-T(2000,1);
% plot_var_etho(Y,seq_c_track.var.B.data(:,:),1,'Displacement Y')
%
% seq_c_track.var.Y.data(:,1) = Y(:,2);
% [seq_c] = crop_seq(seq,1101,10739);  % Crop behavior before plotting
% Y(:,1) = -seq_ref.var.X.data(1101:end-1)+seq_ref.var.X.data(1101);
% Y(:,2) = seq_track.var.Y.data(1101:end)-seq_track.var.Y.data(1101);
% Y(:,3) = T(1101:end,1)-T(1101,1);
% plot_var_etho(Y,seq_c.var.B.data(:,:),1,'Displacement Y')
%
% X(:,1) = -seq_ref.var.Y.data(1101:end-1)+seq_ref.var.Y.data(1101);
% X(:,2) = seq_track.var.X.data(1101:end)-seq_track.var.X.data(1101);
% X(:,3) = T(1101:end,2)-T(1101,2);
% plot_var_etho(X,seq_c.var.B.data(:,:),1,'Displacement X')


%% Geoplot seq

% for i = 1:7
% [seq_track_test] = crop_seq(seq_track,lim(i,1),lim(i,2)-1);
% [seq_ref_test] = crop_seq(seq_ref,lim(i,1),lim(i,2)-1);
%
% [seq_track_test] = seq_ptrack2geo(seq_track_test,seq_ref_test);
% geoplot(seq_track_test.var.LAT.data,seq_track_test.var.LON.data,'g')
% hold off
% geobasemap("satellite");
% title('Exemple of turtle trajectory with acoustic positioning')
% hold on
% geoplot(seq_c_ref(i).var.LAT.data,seq_c_ref(i).var.LON.data,'g*')
% end


%% Geoplot

%
% [seq_track_test] = crop_seq(seq_track,2000,10739);
% [seq_ref_test] = crop_seq(seq_ref,2000,10739);
%
% [seq_track_test] = seq_ptrack2geo(seq_c_track,seq_ref_test);
% geoplot(seq_track_test.var.LAT.data,seq_track_test.var.LON.data,'g')
% hold off
% geobasemap("satellite");
% title('Exemple of turtle trajectory with acoustic positioning')
% hold on
% geoplot(seq_ref_test.var.LAT.data,seq_ref_test.var.LON.data,'g*')


%% Compression and uncertainty


% Analyse

% Variable
s_seq_ref = [];
s_seq = [];
x = [];
x_ref =  [];
y = [];
y_ref =  [];

for i=1:7

    s_ref_buf = seq_c_ref(i,1).var.S.data.Var1(1:end-1,1);
    %s_buf = seq_c_track(i).var.S.data(:,1);
    s_seq_ref = [s_seq_ref; s_ref_buf];
    %s_seq = [s_seq; s_buf];

    x_ref_buf = seq_c_ref(i).var.X.data(1:end-1,1);
    %x_buf = seq_c_track(i).var.X.data(:,1);
    x_ref = [x_ref; x_ref_buf];
    %x = [x; x_buf];

    y_ref_buf = seq_c_ref(i).var.Y.data(1:end-1,1);
    %y_buf = seq_c_track(i).var.Y.data(:,1);
    y_ref = [y_ref; y_ref_buf];
    %y = [y; y_buf];

end
%
% % SPEED
% rmse_s(:,1) = sqrt(mean((s_seq_ref-s_seq).^2));
% rmse_s(:,2) = sqrt(mean((s_seq_ref-s_c).^2))
%
%
% %DISTANCE
% fs = 1;
% len = length(s_seq_ref(:,1));
% disth(1,1:3) =0;
%
% for i=1:len
% disth(1,1) = disth(1,1)+ (s_seq_ref(i,1)/fs);
% disth(1,2) = disth(1,2) + (s_seq(i,1)/fs);
% disth(1,3) = disth(1,3) + (s_c(i,1)/fs);
% end
%
% % 2DRMS
% for i =2:len
%   % TRAJ 1
%   delta_x(i,1) =  (x_ref(i,1)-x_ref(i-1,1)) + (y(i,1)-y(i-1,1));
%   delta_y(i,1) =  (y_ref(i,1)-y_ref(i-1,1)) + (x(i,1)-x(i-1,1));
%   %TRAJ 2
%   delta_x(i,2) =  (x_ref(i,1)-x_ref(i-1,1)) + (y_c(i,1)-y_c(i-1,1));
%   delta_y(i,2) =  (y_ref(i,1)-y_ref(i-1,1)) + (x_c(i,1)-x_c(i-1,1));
%
%
%   % TRAJ 1
%   delta1_x(i,1) =  x_ref(i,1) + y(i,1);
%   delta1_y(i,1) =  y_ref(i,1) + x(i,1);
%
%   % TRAJ 2
%   delta1_x(i,2) =  x_ref(i,1) + y_c(i,1);
%   delta1_y(i,2) =  y_ref(i,1) + x_c(i,1);
%
%
% end
%
% delta_x = filloutliers(delta_x,'linear');
% delta_y = filloutliers(delta_y,'linear');
%
% HRMS(:,1) = sqrt(mean(delta_x(:,1).^2 + delta_y(:,1).^2));
% HRMS(:,2) = sqrt(mean(delta_x(:,2).^2 + delta_y(:,2).^2))
%
% HMRS1(:,1) = sqrt(mean(delta1_x(:,1).^2 + delta1_y(:,1).^2));
% HMRS1(:,2) = sqrt(mean(delta1_x(:,2).^2 + delta1_y(:,2).^2))
%
%
% % Tortuisty
% t(1,:) = tortuosity([x_ref y_ref],1,length(x_ref(:,1)));
% t(2,:) = tortuosity([x(:,1) y(:,1)],1,length(x(:,1)));
% t(3,:) = tortuosity([x_c(:,1) y_c(:,1)],1,length(x(:,1)))





%% FIND AND CREATE GEOLCATED correction value

[GEO_POS] = surf_geo_pos(seq_c_track,seq_c_r);

k = [1:2:size(GEO_POS,1)];%,size(GEO_POS,1)] ;
%Test fit track

D(:,1) = seq_c_track.var.X.data;
D(:,2) = seq_c_track.var.Y.data;

%GEO_POS(k,3) = -GEO_POS(k,3);
[D,C] = fit_tracks(GEO_POS(k,2:3),GEO_POS(k,1),D,1);
[D1,C1] = fit_tracks(GEO_POS(k,2:3),GEO_POS(k,1),[T(7247:10739,2)-T(7247,2),T(7247:10739,1)-T(7247,1)],1);



plot(-y_ref(3584:7076)+y_ref(3584,1),-x_ref(3584:7076)+x_ref(3584,1),'g');
axis equal;
hold on;
plot(D(:,1),D(:,2),'r');
plot(D1(:,1),D1(:,2),'b');
%plot(seq_c_track.var.X.data,seq_c_track.var.Y.data,'b');


% for i =1:200:length(seq_c_track.var.X.data(:,1))
%     plot(seq_c_r.var.X.data(i,1),seq_c_r.var.Y.data(i,1),'xg');
% end
for i =3584:100:7076
    plot(-y_ref(i,1)+y_ref(3584,1),-x_ref(i,1)+x_ref(3584,1),'xg');
end

% for i =1:400:length(seq_c_track.var.X.data(:,1))
%     circle(seq_c_track.var.X.data(i,1),seq_c_track.var.Y.data(i,1),i*0.17/5,'b');
% end


%% Function to make circle growing and reset uncertainty
i=1;
mid_traj(i,1)= floor(GEO_POS(k(i),1)/2);
for i =2:length(k)
    mid_traj(i,1) = floor((GEO_POS(k(i),1)-GEO_POS(k(i-1),1))/2);
end

inc=0;
inc1 = 1;
inc_beha = 1;
len_dive = GEO_POS(k(inc1),1);
r = zeros(length(seq_c_track.var.X.data(:,1)),1);
r(1,1) = 5;
flag_1 =1;

for i =2:length(seq_c_track.var.X.data(:,1))-2
    

    %Length of the sequence corrected
    if inc1 > 1 && inc1 < size(k,2)
        len_dive = (GEO_POS(k(inc1),1)-GEO_POS(k(inc1-1),1));
    end
    
    % New sequence corrected
    if inc1 <= size(k,2)
        
        if i > GEO_POS(k(inc1))            
            inc1 = inc1+1;  
            inc = 1;
            r(i-1,1) = 5;
            flag_1 =1;      
        end
    end
    
    %Find current behavior
    if i > seq_c_track.var.B.data(inc_beha,3)
        inc_beha = inc_beha+1;
    end
    
    c_behavior = seq_c_track.var.B.data(inc_beha,1);  % Current behavior
    
    
    % No more correction apply normal algorithm
    if inc1 > size(k,2)
        
        if c_behavior == 2
            uncert_coeff = 0;
        else
            uncert_coeff = 0.15;
        end
        
        r(i,1) = r(i-1,1)+uncert_coeff;
        
    else
        
        if inc < mid_traj(inc1,1)
            if c_behavior == 2
                uncert_coeff = 0;
            else
                uncert_coeff = 0.10;
            end
            
            r(i,1) = r(i-1,1)+uncert_coeff;
            
        elseif inc >= mid_traj(inc1,1)
            if flag_1 == 1
                flag_1 =0;
                if c_behavior == 2
                    uncert_coeff = 0;
                else
                    %coeff_fct = (10-r(i-1,1))/(mid_traj);
                    uncert_coeff = (10-r(i-1,1))/(GEO_POS(k(inc1),1)-i);
                end
                
            end
            r(i,1) = r(i-1,1)+uncert_coeff;
        end
    end
    
    inc = inc+1;
    
end


% Map circle
for i =1:100:length(seq_c_track.var.X.data(:,1))
    circle(D(i,1),D(i,2),r(i,1),'r');
    hold on
    plot(D(i,1),D(i,2),'xr');
end

plot(GEO_POS(1:15,2),GEO_POS(1:15,3),'xb');


diff_pos(1,2) = GEO_POS(1,1);
for i =1:length(k)
    t(i,1) = GEO_POS(k(i),1);
    diff_pos(i,1) = sqrt( (seq_c_track.var.X.data(t(i,1),1)-D(t(i,1),1)).^2-(seq_c_track.var.Y.data(t(i,1),1)-D(t(i,1),2).^2));
    if i > 1
        diff_pos(i,2) = t(i,1)-t(i-1,1);
    end
end

%% 2DRMS

%DISTANCE
len = length(D(:,1));

x_ref_rms = -y_ref(3584:7076);
y_ref_rms = -x_ref(3584:7076);

T1(:,1) = T(7247:10739,2);
T1(:,2) = T(7247:10739,1);

% D(:,1) = seq_c_track.var.X.data;
% D(:,2) = seq_c_track.var.Y.data;

% 2DRMS
for i =2:len
    % TRAJ 1
    delta_x(i,1) =  (x_ref_rms(i,1)-x_ref_rms(i-1,1)) - (D(i,1)-D(i-1,1));
    delta_y(i,1) =  (y_ref_rms(i,1)-y_ref_rms(i-1,1)) - (D(i,2)-D(i-1,2));    
    % TRAJ 2
    delta1_x(i,1) =  (x_ref_rms(i,1)-x_ref_rms(i-1,1)) - (T1(i,1)-T1(i-1,1));
    delta1_y(i,1) =  (y_ref_rms(i,1)-y_ref_rms(i-1,1)) - (T1(i,2)-T1(i-1,2));  
end


delta_x = filloutliers(delta_x,'linear');
delta_y = filloutliers(delta_y,'linear');

HRMS(:,1) = sqrt(mean(delta_x(:,1).^2 + delta_y(:,1).^2));
HMRS1(:,1) = sqrt(mean(delta1_x(:,1).^2 + delta1_y(:,1).^2));



% 2DRMS V2 
for i =2:len
    % TRAJ 1
    delta_x(i,1) =  x_ref_rms(i,1) - (D(i,1));
    delta_y(i,1) =  y_ref_rms(i,1) - (D(i,2));    
    % TRAJ 2
    delta1_x(i,1) =  x_ref_rms(i,1) - (D1(i,1));
    delta1_y(i,1) =  y_ref_rms(i,1) - (D1(i,2));  
end

delta_x = filloutliers(delta_x,'linear');
delta_y = filloutliers(delta_y,'linear');

HRMS(:,1) = sqrt(mean(delta_x(:,1).^2 + delta_y(:,1).^2));
HMRS1(:,1) = sqrt(mean(delta1_x(:,1).^2 + delta1_y(:,1).^2));




%% TEST
start_w = 7242;
end_w = 10739;
plot(-(waterlinked_filter.y_w(start_w:end_w,1)-waterlinked_filter.y_w(start_w,1))*1000,(waterlinked_filter.x_w(start_w:end_w,1)-waterlinked_filter.x_w(start_w,1))*-1000,'*');
plot(-y_ref{7},-x_ref{7},'*');

plot(T(1050:end,2)-T(1050,2),T(1050:end,1)-T(1050,1),'*')
%plot(T1(:,2),T1(:,1))
%plot(-T(:,2),T(:,1))


% plot_var_etho(Y10(:,1:2),seq_c.var.B.data(:,:),1,'Displacement Y')
[seq_c] = crop_seq(seq,1101,10739);
plot_var_etho(seq.var.O.data(1101:end,1),seq_c.var.B.data(:,:),1,'ODBA')
plot_var_etho(seq_c.var.P.data,seq_c.var.B.data(:,:),1,'ODBA')
% X(:,1) = seq.var.X.data(600:end,1);
% X(:,2) = -seq_ref.var.Y.data(600:end-1,1)+seq_ref.var.Y.data(600);
% Y(:,1) = seq.var.Y.data(600:end,1);
% Y(:,2) = -seq_ref.var.X.data(600:end-1,1)+seq_ref.var.Y.data(600);
plot_var_etho(X(:,1:2),seq_c.var.B.data(:,:),1,'Displacement X')
plot_var_etho(Y(:,1:2),seq_c.var.B.data(:,:),1,'Displacement Y')
plot_var_etho(seq_c.var.S.data(:,1),seq_c.var.B.data(:,:),1,'Speed (m/s)')
