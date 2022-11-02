% Time range of analysis
limit_analyse_1 = "2021-14-12 13:51:00"; %début : 200m avant la pointe
limit_analyse_2 = "2021-14-12 17:50:00"; % Fin : Après la boucle et la pause

%% IMU offset
% 18/08 Lagon paddle
%load('imu_offset.mat')
magoffset = [-0.258290949377437,0.633233609989412,-0.539841341098975];
gyrooffset = [-2.302746151840946,-1.490113616615377,-0.861883223041896];
acceloffset = [0,0,0];

srate = [100 10 1];

%% Import file
% Openlogger variables
compute_data_logger = 1;
seconds_offset_openlogger = -127;
plot_raw_data_logger = 0;

%Script to merge raw data
merge_data_raw

% Compute raw data
[data_tag, data, duration] = compute_openLogger_time_2(srate,data_raw,limit_analyse_1,limit_analyse_2,seconds_offset_openlogger,plot_raw_data_logger,magoffset,gyrooffset,acceloffset);

clear data_raw data_raw_1 data_raw_2 data_raw_3 data_raw_4 data_raw_5 data_raw_6

depth_raw_1hz = zeros(length(data.Time(:,1))/100,1);
inc = 1;
for i =1:100:length(data.depth(:,1))
    depth_raw_1hz(inc,1) = data.depth(i,1);
    inc = inc + 1;
end

