
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

clearvars -except seq_s_ref seq_ref var seq_buf waterlinked_filter waterlinked speed_no_filter

%% Proccessing ALGO
% Proccessing ALGO 1 (filtering, variables, dive, ethogram)

processing_algo1

%% Crop by sequence

for i=1:length(seq_s_ref(:,1))
    lim_inf = seq_s_ref(i,1).tstart;
    lim_sup = seq_s_ref(i,1).tend-1;
    [seq_s(i,1)] = crop_seq(seq,lim_inf,lim_sup);
end


%% Plot 



nb_seq = 8;

%[seq_track_test] = seq_ptrack2geo(seq_s(nb_seq),seq_s_ref(nb_seq));


figure(11)

t = tiledlayout(4,5,'Padding','compact');%,'TileSpacing' ,'compact');


geop = geoaxes(t);
geop.Layout.Tile = 1;
geop.Layout.TileSpan = [4 2];
bev = seq_s(nb_seq).var.B.data(:,:);

for i = 1:length(bev(:,1))
    
    if bev(i,1) == 1
        box_color(i,1:3) =  [0.87 0.79 0.99];
    elseif bev(i,1) == 2
        box_color(i,1:3) =  [0.39,0.39,0.39];
    elseif bev(i,1) == 3
        box_color(i,1:3) =  [0.40,1.00,0.25];
    elseif bev(i,1) == 4
        box_color(i,1:3) =  [0.98,0.20,0.20];
    elseif bev(i,1) == 5
        box_color(i,1:3) =  [0.22,0.29,0.98];
    elseif bev(i,1) == 6
        box_color(i,1:3) =  [1.00,0.84,0.22];
    else
        box_color(i,1:3) =  [0 1 1];
    end
    
 
    geoplot(geop,seq_s_ref(nb_seq,1).var.LAT.data(bev(i,2):bev(i,3)),seq_s_ref(nb_seq,1).var.LON.data(bev(i,2):bev(i,3)),'Color',box_color(i,1:3),'LineWidth',2);

    hold on
% 
%     
%     plot(-y{seq}(roi{seq}(i,1):roi{seq}(i,2),1),-x{seq}(roi{seq}(i,1):roi{seq}(i,2),1),'Color',box_color(i,1:3));
%     hold on
%     plot(-y{seq}(roi{seq}(i,1):3:roi{seq}(i,2),1),-x{seq}(roi{seq}(i,1):3:roi{seq}(i,2),1),'*','Color',box_color(i,1:3));

%     plot(x{seq}(roi{seq}(i,1):roi{seq}(i,2),1),-y{seq}(roi{seq}(i,1):roi{seq}(i,2),1),'Color',box_color(i,1:3));
%     hold on
%     plot(x{seq}(roi{seq}(i,1):3:roi{seq}(i,2),1),-y{seq}(roi{seq}(i,1):3:roi{seq}(i,2),1),'*','Color',box_color(i,1:3));
%     
    
end
%     goep = gca;
geop.LatitudeAxis.TickLabelRotation = 90;
geop.Basemap = 'satellite';
hold off
box on
% axis tight
% axis equal
% ylabel('East-West (m)')
% xlabel('North-South (m)')
title('Turtle trajectory')




 etho_color(1,1:3) =  [0.87 0.79 0.99];
 etho_color(5,1:3) =  [0.39,0.39,0.39];
 etho_color(3,1:3) =  [0.40,1.00,0.25];
 etho_color(4,1:3) =  [0.98,0.20,0.20];
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
c.Location = 'eastoutside';
c.TickLabels = tickLbls;
c.Ticks = Ticks;
c.TickLength = 0;

nexttile([2 3])
plot_var_etho_1(seq_s(7).var.P.data(:,1)*-1,seq_s(7).var.B.data(:,:),1,'Depth (m)')

nexttile([1 3])
plot_var_etho_1_B2_reg(seq_s(7).var.Ad.data(:,1),seq_s(7).var.B.data(:,:),seq_s(7).var.B2.data(:,:),1,'Acc X')
xlim([1900 2104])

nexttile([1 3])
plot_var_etho_1_B2_pitch(seq_s(7).var.Pi.data(:,1)*180/pi,seq_s(7).var.B.data(:,:),seq_s(7).var.B2.data(:,:),1,'Pitch')
xlim([1900 2104])
grid on

% 
% % Create arrow
% annotation(figure1,'arrow',[0.431715210355987 0.223300970873786],...
%     [0.441717791411043 0.328220858895706],'LineWidth',4,'HeadWidth',20,...
%     'HeadLength',20);
% 
% % Create arrow
% annotation(figure1,'arrow',[0.432362459546926 0.214886731391586],...
%     [0.0950920245398773 0.144171779141104],'LineWidth',4,'HeadWidth',20,...
%     'HeadLength',20);


