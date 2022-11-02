

%% Sequence limits
main_manual_etho


%% Waterlinked 

merge_waterlinked_data
% Time range of analysis
limit_analyse_1 = "2021-14-12 12:51:00"; %début : 200m avant la pointe
limit_analyse_2 = "2021-14-12 16:50:00"; % Fin : Après la boucle et la pause
t1 = datetime(limit_analyse_1,'InputFormat','yyyy-dd-MM HH:mm:ss');
t2 = datetime(limit_analyse_2,'InputFormat','yyyy-dd-MM HH:mm:ss');
TR = timerange(t1,t2);
waterlinked  = waterlinked(TR,:);
waterlinked = retime(waterlinked,'regular','fillwithmissing','TimeStep',seconds(1));
%waterlinked_10 = retime(waterlinked,'regular','linear','TimeStep',seconds(0.1))


%% Traj and Speed reference 
% waterlinked STD filter 
std_limit = 3;
waterlinked_filter = waterlinked;

for i=1:length(waterlinked.Time(:,1))
    
    if waterlinked.std(i,1) <= std_limit && waterlinked.std(i,1) > 0
       waterlinked_filter(i,:) = waterlinked(i,:);
    else
       waterlinked_filter.lat_w(i,1) = NaN;
       waterlinked_filter.lon_w(i,1) = NaN;
       waterlinked_filter.lat_g(i,1) = NaN;
       waterlinked_filter.lon_g(i,1) = NaN;
       
       waterlinked_filter.x_w(i,1) = NaN;
       waterlinked_filter.y_w(i,1) = NaN;
       waterlinked_filter.z_w(i,1) = NaN;
       
       waterlinked_filter.x_g(i,1) = NaN;
       waterlinked_filter.y_g(i,1) = NaN;
       waterlinked_filter.z_g(i,1) = NaN;
       
       waterlinked_filter.std(i,1) = NaN;
    end

        
end


for i=1:length(lim(:,1))
    x_t1{i} = (waterlinked.x_w(lim(i,1):lim(i,2))-waterlinked.x_w(lim(i,1),1))*1000;
    y_t1{i} = (waterlinked.y_w(lim(i,1):lim(i,2))-waterlinked.y_w(lim(i,1),1))*1000;
    %z_t = resample(waterlinked.z_w, 1:length(waterlinked.z_w(lim_inf:lim_sup,1)));
 
    x_t{i} = (waterlinked_filter.x_w(lim(i,1):lim(i,2))-waterlinked_filter.x_w(lim(i,1),1))*1000;
    y_t{i} = (waterlinked_filter.y_w(lim(i,1):lim(i,2))-waterlinked_filter.y_w(lim(i,1),1))*1000;
    
    lat_ref{i} = waterlinked_filter.lat_w(lim(i,1):lim(i,2));
    lon_ref{i} = waterlinked_filter.lon_w(lim(i,1):lim(i,2));
    % %z_t = resample(waterlinked.z_w, 1:length(waterlinked.z_w(lim_inf:lim_sup,1)));
    %z{i} = depth_raw_1hz(lim_inf:lim_sup);
    
    % Methode 2 : Resample simple
    x_ref1{i}(:,1) = resample(x_t{i}, 1:length(x_t{i}(:,1)));
    y_ref1{i}(:,1) = resample(y_t{i}, 1:length(y_t{i}(:,1)));
    x_t2{i}(:,1) = smoothdata(x_ref1{i}(:,1),'movmean',60);
    y_t2{i}(:,1) = smoothdata(x_ref1{i}(:,1),'movmean',60);
    x_ref{i}(:,1) = smoothdata(x_ref1{i}(:,1),'movmean',33);   % Normally 10
    y_ref{i}(:,1) = smoothdata(y_ref1{i}(:,1),'movmean',33);

    
    lat_ref{i}(:,1) = resample(lat_ref{i}, 1:length(lat_ref{i}(:,1)));
    lon_ref{i}(:,1) = resample(lon_ref{i}, 1:length(lon_ref{i}(:,1)));
    lat_w{i}(:,1) =  smoothdata(lat_ref{i}(:,1),'movmean',10);
    lon_w{i}(:,1) =  smoothdata(lon_ref{i}(:,1),'movmean',10);
    
end



for j=1:length(lim(:,1))
    
    for i=2:length(x_ref{j}(:,1))
        speed_1hz{j}(i,1) = sqrt((x_ref{j}(i,1)-x_ref{j}(i-1,1)).^2+(y_ref{j}(i,1)-y_ref{j}(i-1,1)).^2);
        speed_no_filter{j}(i,1) = sqrt((x_ref1{j}(i,1)-x_ref1{j}(i-1,1)).^2+(y_ref1{j}(i,1)-y_ref1{j}(i-1,1)).^2);
        speed_no_filter{j}(i,2) = sqrt((x_t{j}(i,1)-x_t{j}(i-1,1)).^2+(y_t{j}(i,1)-y_t{j}(i-1,1)).^2);
        speed_no_filter{j}(i,3) = sqrt((x_t2{j}(i,1)-x_t2{j}(i-1,1)).^2+(y_t2{j}(i,1)-y_t2{j}(i-1,1)).^2);
        
        disp_x{j}(i,1) = (x_ref{j}(i,1)-x_ref{j}(i-1,1));
        disp_y{j}(i,1) = (y_ref{j}(i,1)-y_ref{j}(i-1,1));
        disp_x{j}(i,2) = (x_ref1{j}(i,1)-x_ref1{j}(i-1,1));
        disp_y{j}(i,2) = (y_ref1{j}(i,1)-y_ref1{j}(i-1,1));
        disp_x{j}(i,3) = disp_x{j}(i,1);
        disp_y{j}(i,3) = disp_y{j}(i,1);

   
    end
    
    %speed_no_filter{j}(:,4) =  smoothdata(speed_no_filter{j}(:,1),'movmean',33);
    %speed_1hz{j}(:,1) = smoothdata(speed_no_filter{j}(:,1),'movmean',33);
    time{j} = seconds(1:length(speed_1hz{j}(:,1)));
    speed_ref_1{j} = timetable(time{j}',speed_1hz{j}(:,:));
        
        
    speed_ref_10{j} = retime(speed_ref_1{j},'regular','linear','TimeStep',seconds(0.1));
    speed_ref_10{j}.Time = speed_ref_10{j}.Time+seconds(lim(j,1));
    %data_speed_10{j} = data_speed_10{j}(1:length(data_speed_model_10{j}(:,1)),:);
end

for j=1:length(ethogram)
    
    speed_filter_3{j} = speed_ref_1{j};
    
    for i=1:length(ethogram_name{j}(:,1))-1
        
        if strcmp(bev{j}(i,1) , 'rest') %|| strcmp(bev{j}(i,1), 'm_rest') %|| strcmp(bev{j}(i,1) , 'p_rest')
            
            
            speed_ref_10{j}.Var1(roi{j}(i,1)*10:roi{j}(i,2)*10,1) = 0;
            speed_ref_1{j}.Var1(roi{j}(i,1):roi{j}(i,2),1) = 0;
            %data_speed_10{j}.Var1(roi{j}(i,1)*10:roi{j}(i,2)*10,3) = 0;
             x_ref{j}(roi{j}(i,1):roi{j}(i,2),1) = x_ref{j}(roi{j}(i,1),1);
             y_ref{j}(roi{j}(i,1):roi{j}(i,2),1) = y_ref{j}(roi{j}(i,1),1);
             
            disp_x{j}(roi{j}(i,1):roi{j}(i,2),3) = 0;
            disp_y{j}(roi{j}(i,1):roi{j}(i,2),3) = 0;
            
        end
        
    end

end

for i=1:length(lim(:,1))
    data_c_ref(i,1).X = x_ref{i};
    data_c_ref(i,1).Y = y_ref{i};
    data_c_ref(i,1).S = speed_ref_1{i};
    data_c_ref(i,1).S10 = speed_ref_10{i};
    data_c_ref(i,1).LAT = lat_w{i};
    data_c_ref(i,1).LON = lon_w{i};
    data_c_ref(i,1).lim_inf = lim(i,1);
    data_c_ref(i,1).lim_sup = lim(i,2);
    data_c_ref(i,1).time = 'sec';
end

data_ref.X = (waterlinked_filter.x_w-waterlinked_filter.x_w(lim(i,1)))*1000;
data_ref.Y = (waterlinked_filter.y_w-waterlinked_filter.y_w(lim(i,1)))*1000;
data_ref.LAT = waterlinked_filter.lat_w;
data_ref.LON = waterlinked_filter.lon_w;
data_ref.lim_inf = 0;
data_ref.lim_sup = lim(8,2);
data_ref.time = 'sec';
