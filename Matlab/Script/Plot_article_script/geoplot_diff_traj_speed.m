clear all
close all

% Import tag data in timetables
main_import_tag_data

%Script for reference data
main_ref_seq

%% format data tag
[var] = f_tt2var(data_tag,100);
[seq_buf] = f_var2seq(var);

%% Format data
len_s = length(data_ref(:)); % Number of valid sequence for validation

[seq_s_ref] = f_tt2seq_ref(data_c_ref,1); %Store all the ref sequence
[seq_ref] = f_tt2seq_raw_ref(data_ref,1); %Store all the ref sequence

clearvars -except seq_s_ref seq_ref var seq_buf

%% Proccessing ALGO
% Proccessing ALGO 1 (filtering, variables, dive, ethogram)

processing_algo1_bis

% Proccessing ALGO 2 (filtering, variables, dive, ethogram)

processing_algo2

% Proccessing ALGO 3 (Mark Johnson)

processing_algo3_bis

% Proccessing ALGO 3 (Mark Johnson)

processing_algo4  %Nb 4


%% calculate % of behavior

%[timing_1, timing_2] = behavior_timing(seq(1,1));


% % 
% clear A B x 
% A(:,1) = seq(1,1).var.B2.data(:,4);
% A(:,2) = seq(1,1).var.B2.data(:,3);
% 
% for i=1:length(A(:,1))
%     if A(i,1)==0
%         A(i,1:2) = NaN;
%     end
% end
% 
% % xx = fillgaps(A(:,1),80,40);
% % xx(:,2) = fillgaps(B(:,1),80,40);
% 
% load('model_accx.mat')
% load('speed_accx_1.mat')
% 
% for i=1:length(seq(1,1).var.S.data(:,1))
%     if seq(1,1).var.B2.data(i,2) == 1
%        seq(1,1).var.S.data(i,1) = trainedModel_raw_1.predictFcn([A(i,:), A(i,:)]); 
%     end
% end
% 
% %seq(1,1).var.S.data(:,1) = trainedModel_accx.predictFcn(xx(:,:)); 
% seq(1,1).var.S.data( seq(1,1).var.B2.data(:,1) == 2) = 0;
% seq(1,1).var.S.data( seq(1,1).var.B2.data(:,1) == 6) = 0;
% 
% for i=1:length(seq(1,1).var.S.data(:,1))-1
%     if ~isnan(speed_pitch(i,1))
%     seq(1,1).var.S.data(i,1) = abs(speed_pitch(i,1));
%     else
%     seq(1,1).var.S.data(i,1) = seq(1,1).var.S.data(i,1);    
%     end
% end
% 
% seq(1,1).var.S.data(:,1) = smoothdata(seq(1,1).var.S.data(:,1),'movmean',15)*0.9;
% %seq(1,1).var.S.data(:,1) = seq(1,1).var.S.data(:,1)*0.95;

%% Crop seq to match reference seq and calculate trajectories


for j=1:4
    
    buf_in = struct(seq(j,1));
    % Crop sequence with ref sequence limits
    for i=1:length(seq_s_ref(:,1))
        lim_inf = seq_s_ref(i,1).tstart;
        lim_sup = seq_s_ref(i,1).tend-1;
        [buf_in_s(i,j)] = crop_seq(buf_in,lim_inf,lim_sup);
    end
    
    % Calculate trajectory
    offset_h = -19.32;%-19.32;
    for i=1:length(seq_s_ref(:,1))
        [seq_s(i,j)] = seq_seq2htrack(buf_in_s(i,j),offset_h);    
    end
end


j=5;
for i=1:length(seq_s_ref(:,1))
        lim_inf = seq_s_ref(i,1).tstart;
        lim_sup = seq_s_ref(i,1).tend-1;
        [buf_in_s(i,j)] = crop_seq(buf_in,lim_inf,lim_sup);
        
        buf_in_s(i,j).var.S.data = seq_s_ref(i,1).var.S.data.Var1(1:end-2);
        
        [seq_s(i,j)] = seq_seq2htrack(buf_in_s(i,j),offset_h);    
end


[seq] = seq_seq2htrack(seq,offset_h);
clear T buf_in buf_in_s

 
%% Concat variable for analyse

% Variable
s_seq_ref = [];
s_seq = [];
x = [];
x_ref =  [];
y = [];
y_ref =  [];

for i=8:8
    
    if i ~= 6
        
    s_ref_buf = seq_s_ref(i).var.S.data.Var1(1:end-2,1);
    x_ref_buf = -seq_s_ref(i).var.Y.data(1:end-2,1);
    y_ref_buf = -seq_s_ref(i).var.X.data(1:end-2,1);
    
    s_seq_ref = [s_seq_ref; s_ref_buf];
    x_ref = [x_ref; x_ref_buf];
    y_ref = [y_ref; y_ref_buf];
    
    for j=1:5
        if j ==1
            s_buf(:,j) = seq_s(i,j).var.S.data(:,1);
            y_buf(:,j) = seq_s(i,j).var.Y.data(:,1);
            x_buf(:,j) = seq_s(i,j).var.X.data(:,1);
        elseif j==3
            s_buf(:,j) = seq_s(i,j).var.S.data(:,2);
            y_buf(:,j) = seq_s(i,j).var.Y.data(:,1);
            x_buf(:,j) = seq_s(i,j).var.X.data(:,1);
        elseif j==2
            s_buf(:,j) = seq_s(i,j).var.S.data(:,2);
            y_buf(:,j) = seq_s(i,j).var.Y.data(:,1);
            x_buf(:,j) = seq_s(i,j).var.X.data(:,1);
        else
            s_buf(:,j) = seq_s(i,j).var.S.data(:,1);
            y_buf(:,j) = seq_s(i,j).var.Y.data(:,1);
            x_buf(:,j) = seq_s(i,j).var.X.data(:,1);
        end
        
    end
   
    
    y = [y; y_buf];
    x = [x; x_buf];
    s_seq = [s_seq; s_buf];
    clear s_buf x_buf y_buf
    
    end
    
end

%plot_var_etho([-seq_s_ref(7,1).var.X.data(1:end-2), seq_s(7,1).var.Y.data, seq_s(7,3).var.Y.data],seq_s(7,1).var.B.data,1,'Displacement X')
%plot_var_etho([-seq_s_ref(2,1).var.X.data(1:end-2), seq_s(2,1).var.Y.data, seq_s(2,3).var.Y.data],seq_s(2,1).var.B.data,1,'Displacement X')

%plot_var_etho([-seq_s_ref(7,1).var.Y.data(1:end-2), seq_s(7,1).var.X.data, seq_s(7,3).var.X.data],seq_s(7,1).var.B.data,1,'Displacement X')

clear x_ref_buf y_ref_buf s_ref_buf



%% Plot traj

for i=1:length(seq_s_ref(:,1))
    lim_inf = seq_s_ref(i,1).tstart;
    lim_sup = seq_s_ref(i,1).tend;
    seq_s_ref_raw(i,1).var.X.data = seq_ref.var.X.data(lim_inf:lim_sup,1);
    seq_s_ref_raw(i,1).var.Y.data = seq_ref.var.Y.data(lim_inf:lim_sup,1);
end





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
for i=1:length(seq(:,1))
[seq_c_track(i,1)] = crop_seq(seq(i,1),10741,12753);  %SEQ8
end
[seq_c_r] = crop_seq(seq_ref,10741,12753);  %SEQ8


for i = 1:length(seq_c_track(:,1))
[seq_track_test(i,1)] = seq_ptrack2geo(seq_c_track(i,1),seq_c_r(1,1));
end

[seq_track_test(5,1)] = seq_ptrack2geo(seq_s(8,5),seq_c_r(1,1));


  
%% PLOT 1


figure1 = figure(1);

%t = tiledlayout('tight');
t = tiledlayout(4,6,'Padding','compact','TileSpacing' ,'compact');


nexttile([4 2])
geoplot(seq_track_test(1,1).var.LAT.data,seq_track_test(1,1).var.LON.data,'b','LineWidth',2)
hold on
geoplot(seq_track_test(2,1).var.LAT.data,seq_track_test(2,1).var.LON.data,'y','LineWidth',2)
geoplot(seq_track_test(3,1).var.LAT.data,seq_track_test(3,1).var.LON.data,'r','LineWidth',2)
geoplot(seq_track_test(4,1).var.LAT.data,seq_track_test(4,1).var.LON.data,'m','LineWidth',2)
geobasemap("satellite");
title('a) Different estimated trajectories for sequence 7','FontSize',14)

geoplot(seq_s_ref(8,1).var.LAT.data,seq_s_ref(8,1).var.LON.data,'g')
%geoplot(seq_c_r(1,1).var.LAT.data,seq_c_r(1,1).var.LON.data,'g')
%geoplot(seq_track_test(5,1).var.LAT.data,seq_track_test(5,1).var.LON.data,'g*')


legend1 = legend('F1','F2','F3','F4','Reference','AutoUpdate','off');

set(legend1,'TextColor',[1 1 1],'Location','northwest','FontWeight','bold','FontSize',16,...
    'Color','none',...
    'AutoUpdate','off');


nexttile([1 4])
plot_var_etho_1(seq_s(8).var.P.data(:,1)*-1,seq_s(8).var.B.data(:,:),1,'Depth (m)')
xlabel('Time (s)')
ylabel('Depth (m)')
ylim([-15 0])
%legend2 = legend('Dive profile','AutoUpdate','off');
%set(legend2,'Location','northwest','FontWeight','bold','FontSize',14,...
%    'AutoUpdate','off');
title('b) Depth profile of sequence 7 mapped with ethogram','FontSize',14)




nexttile([1 4])
plot(abs(seq_s_ref(8, 1).var.S.data.Var1(1:end-2,1)-seq_s(8, 3).var.S.data(:,1)),'r','LineWidth',2)
hold on
plot_var_etho_1_no_new(abs(seq_s_ref(8, 1).var.S.data.Var1(1:end-2,1)-seq_s(8).var.S.data(:,1)),seq_s(8).var.B.data(:,:),1,'Speed (m/s)')
%plot(seq_s(8, 1).var.S.data(:,1),'b','LineWidth',2)
%plot(seq_s_ref(8, 1).var.S.data.Var1(:,1),'g','LineWidth',2)
xlim([1110 1550])
xlabel('Time (s)')
ylabel('Speed (m/s)')
% legend3 = legend('Speed F1','Speed F3','Speed ref','AutoUpdate','off');
% set(legend3,'Location','northwest','FontWeight','bold','FontSize',14,...
%     'AutoUpdate','off');
title('c) Speed estimation for F1 et F3 compared to reference speed mapped with ethogram','FontSize',14)




nexttile([1 4])
plot_var_etho_1_B2_reg(seq_s(8).var.O.data(:,1),seq_s(8).var.B.data(:,:),seq_s(8).var.B2.data(:,:),1,'Dynamic Acc X (g)')
xlabel('Time (s)')
ylabel('Dynamic Acc X (g)')
xlim([1110 1550])
ylim([0 0.09])
%legend2 = legend('Filtered displacement','AutoUpdate','off');
%set(legend2,'Location','northwest','FontWeight','bold','FontSize',14,...
%    'AutoUpdate','off');
title('d) Dynamic acceleration on X for sequence 7 with regular phases (blue crosses)','FontSize',14)


nexttile([1 4])
plot_var_etho_1_B2_pitch(seq_s(8).var.Pi.data(:,1)*180/pi,seq_s(8).var.B.data(:,:),seq_s(8).var.B2.data(:,:),1,'Pitch')
xlabel('Time (s)')
ylabel('Pitch (°)')
xlim([1110 1550])
ylim([-60 40])

%legend2 = legend('Filtered displacement','AutoUpdate','off');
%set(legend2,'Location','northwest','FontWeight','bold','FontSize',14,...
%    'AutoUpdate','off');
title('e) Pitch for sequence 7 with high pitch phases (Red and green crosses)','FontSize',14)


bev = seq_s(7).var.B.data(:,:);

 etho_color(1,1:3) =  [0.87 0.79 0.99];
 etho_color(5,1:3) =  [0.39,0.39,0.39];
 etho_color(3,1:3) =  [220, 237, 200]/255;%[0.40,1.00,0.25];
 etho_color(4,1:3) =  [255, 205, 210]/255;%[0.98,0.20,0.20];
 etho_color(2,1:3) =  [0.8500 0.3250 0.0980];
 etho_color(6,1:3) =  [1.00,0.84,0.22];
 %etho_color(7,1:3) =  [1 1 1];

 tickLbls(3,1) = {'UP'};
 tickLbls(4,1) = {'DOWN'};
 tickLbls(6,1) = {'GROUND'};
 tickLbls(2,1) = {'SURFACE'};
 tickLbls(5,1) = {'REST'};
 tickLbls(1,1) =  {'SWIM'};
 %tickLbls(7,1) = {'other'};
 
 
%clrs = lines(numel(unique(behavior))-1);
clrs = etho_color;
cmap = fliplr(clrs);
%tickLbls = categories(unique(behavior_s{sequence_nb}))';
% tickLbls = tickLbls';
colormap(gca,clrs);
numCategories = numel(unique(bev(:,1)));
Ticks = 1/(numCategories*2):1/numCategories:1;
c = colorbar;
c.Location = 'southoutside';
c.TickLabels = tickLbls;
c.Ticks = Ticks;
c.TickLength = 0;



