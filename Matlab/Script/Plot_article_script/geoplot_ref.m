clear all


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

clearvars -except seq_s_ref seq_ref var seq_buf waterlinked_filter waterlinked speed_no_filter  speed_filter_3 disp_x disp_y

%% Proccessing ALGO
% Proccessing ALGO 1 (filtering, variables, dive, ethogram)

processing_algo1

%% Crop by sequence

for i=1:length(seq_s_ref(:,1))
    lim_inf = seq_s_ref(i,1).tstart;
    lim_sup = seq_s_ref(i,1).tend-1;
    [seq_s(i,1)] = crop_seq(seq,lim_inf,lim_sup);
end

    
%% PLOT 1


figure1 = figure(1);

%t = tiledlayout('tight');
t = tiledlayout(4,5,'Padding','compact','TileSpacing' ,'compact');


nexttile([4 2])

geoplot(waterlinked_filter.lat_w(:,1),waterlinked_filter.lon_w(:,1),'r','LineWidth',2)
hold on
for i= 1:length(seq_s_ref(:,1))
    geoplot(seq_s_ref(i,1).var.LAT.data,seq_s_ref(i,1).var.LON.data,'g','LineWidth',2);
end
legend1 = legend('Raw position','Filtered positions','AutoUpdate','off');

set(legend1,'TextColor',[1 1 1],'Location','northwest','FontWeight','bold','FontSize',16,...
    'Color','none',...
    'AutoUpdate','off');


geoplot(waterlinked_filter.lat_w(:,1),waterlinked_filter.lon_w(:,1),'r','LineWidth',2)
geoplot(waterlinked_filter.lat_w(:,1),waterlinked_filter.lon_w(:,1),'r*','MarkerSize',4)
for i= 1:length(seq_s_ref(:,1))
    geoplot(seq_s_ref(i,1).var.LAT.data,seq_s_ref(i,1).var.LON.data,'g','LineWidth',2);
end

%geoplot(waterlinked.lat_g(:,1),waterlinked.lon_g(:,1),'y*')
hold off
geobasemap("satellite");
title('a) Turtle trajectory with raw and filtered acoustic positioning')

% 
% %nexttile
% %t = tiledlayout(2,1);
% gax = geoaxes(t);
% gax.Layout.Tile = 3;
% gax.Layout.TileSpan = [2 1];
% gax.MapCenter = [-21.060069226354084, 55.218151877600505];
% %gax.ZoomLevel = 21.374999999714404;
% 
% geoplot(gax,waterlinked_filter.lat_w(:,1),waterlinked_filter.lon_w(:,1),'r*')
% hold on
% for i= 1:length(seq_s_ref(:,1))
%     geoplot(gax, seq_s_ref(i,1).var.LAT.data,seq_s_ref(i,1).var.LON.data,'g');
% end
% 
% geobasemap("satellite");
% %geoplot(gax,[47.62 61.20],[-122.33 -149.90],'g-*')
% 
% 
% nexttile([2 2])
% plot(double(seq.var.He.data*180/pi),'g','LineWidth',2)
% xlim([2140 2300])
% xlabel('Time (s)')
% ylabel('Heading (°)')
% legend2 = legend('Filtered heading','Raw heading','AutoUpdate','off');
% set(legend2,'Location','northwest','FontWeight','bold','FontSize',12,...
%     'Color','none',...
%     'AutoUpdate','off');
% title('b) Heading from filtered reference and IMU data')


%nexttile
%t = tiledlayout(2,1);
gax = geoaxes(t);
gax.Layout.Tile = 3;
gax.Layout.TileSpan = [2 1];
gax.MapCenter = [-21.060069226354084, 55.218151877600505];
%gax.ZoomLevel = 21.374999999714404;

geoplot(gax,waterlinked_filter.lat_w(:,1),waterlinked_filter.lon_w(:,1),'r*')
hold on
for i= 1:length(seq_s_ref(:,1))
    geoplot(gax, seq_s_ref(i,1).var.LAT.data,seq_s_ref(i,1).var.LON.data,'g');
end

geobasemap("satellite");
%geoplot(gax,[47.62 61.20],[-122.33 -149.90],'g-*')


nexttile([2 2])

plot(disp_x{1,7}(:,2),'r','LineWidth',2);
hold on
plot(disp_x{1,7}(:,1),'g','LineWidth',2)

xlim([0 1100])
ylim([-1.6 1.6])
xlabel('Time (s)')
ylabel('Displacement X axis (m)')
legend3 = legend('Raw speed','Filtered speed','AutoUpdate','off');
set(legend3,'Location','northwest','FontWeight','bold','FontSize',14,...
    'AutoUpdate','off');
title('b) Zoom on X axis displacement for raw and filtered reference before zero padding step','FontSize',14)


nexttile([2 3])

plot(disp_x{1,7}(:,2),'r','LineWidth',2)
hold on
plot(disp_x{1,7}(:,3),'g','LineWidth',2);

ylim([-2 2])
xlabel('Time (s)')
ylabel('Displacement X axis (m)')
legend2 = legend('Raw displacement','Filtered displacement','AutoUpdate','off');
set(legend2,'Location','northwest','FontWeight','bold','FontSize',14,...
    'AutoUpdate','off');
title('c) Raw and filtered reference X displacement after zero padding step','FontSize',14)



% Create rectangle
annotation(figure1,'rectangle',...
    [0.228125 0.453790242519705 0.0208333333333333 0.0913810970441575],...
    'LineWidth',3);

% Create rectangle
annotation(figure1,'rectangle',...
    [0.1859375 0.329179646936656 0.0166666666666667 0.0654205607476634],...
    'LineWidth',3);

% Create arrow
annotation(figure1,'arrow',[0.438541666666667 0.247395833333333],...
    [0.923156801661475 0.544132917964694],'LineWidth',3);

% Create arrow
annotation(figure1,'arrow',[0.436458333333333 0.250520833333333],...
    [0.539979231568017 0.455867082035306],'LineWidth',3);

% Create arrow
annotation(figure1,'arrow',[0.439065 0.203125],...
    [0.484942886812046 0.39356178608515],'LineWidth',3);

% Create arrow
annotation(figure1,'arrow',[0.4390625 0.203645833333333],...
    [0.114226375908619 0.330218068535825],'LineWidth',3);




