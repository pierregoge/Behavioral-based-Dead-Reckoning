clear all
close all

addpath(genpath('../../Matlab'))
addpath(genpath('../../Data'))

%Import tag data in timetables
main_import_tag_data

%Script for reference data
main_ref_seq

%% format data tag
[var] = f_tt2var(data_tag,100);
[seq_buf] = f_var2seq(var);

%% Format data ref
len_s = length(data_ref(:)); % Number of valid sequence for validation

[seq_s_ref] = f_tt2seq_ref(data_c_ref,1); %Store all the ref sequence
[seq_ref] = f_tt2seq_raw_ref(data_ref,1); %Store all the ref sequence

clearvars -except seq_s_ref seq_ref var seq_buf

%% Proccessing ALGO
% Proccessing ALGO 1 (10Hz, filtering, variables, dive, ethogram)

processing_algo1 % seq n°1

% Proccessing ALGO 2 (1Hz, filtering, variables, dive, ethogram)

processing_algo2 % seq n°2

% Proccessing ALGO 3 (Animals Tool box functions)

processing_algo3  % seq n°3

% Proccessing ALGO 4 (ODBA regression)

processing_algo4  % seq n°4


%% Calculate percentage of behavior

behavior_timing(seq(2,1));

%% Crop seq to match reference seq and calculate trajectories

crop_and_traj

%% Concat variable for analyse

start_s = 2; %First sequence to concat and analyse
end_s = 8;  %Last sequence to concat and analyse

concat_var

%% Analyse

nb = 5;

% Set all the speed to 0 when the reference speed is 0. This reduce the
% speed accuracy gain from adaptative algo by comparing only a
for i = 2:length(s_seq_ref(:,1))
    if s_seq_ref(i,1) == 0
        s_seq(i,:) = 0;
    end
end

% SPEED
RMSE_S(:,1:nb) = sqrt(mean((s_seq(:,5)-s_seq(:,:)).^2));

%DISTANCE
fs = 1;
len = length(s_seq_ref(:,1));
disth(1,1:nb+1) =0;

for i=1:len
    disth(1,1) = disth(1,1)+ (s_seq_ref(i,1)/fs);
    disth(1,2:nb+1) = disth(1,2:nb+1) + (s_seq(i,1:nb)/fs);
end

% 2DRMS
nb = 4;
for i =2:len
    delta_x(i,1:nb) =  x(i,5) - x(i,1:nb);
    delta_y(i,1:nb) =  y(i,5) - y(i,1:nb);   
end


inc = 0;
for i =2:len
    if s_seq(i,3)  ~= 0
        inc = inc + 1;
    end
end

nb = 4;
HMRS(:,1:nb) = sqrt(mean(delta_x(:,1:nb).^2 + delta_y(:,1:nb).^2));

% RESULT
RESULT(1,:) = RMSE_S(1,1:4);
RESULT(2,:) = HMRS(1,1:4);
RESULT(3,:) = abs(disth(1,1)-disth(1,2:5));



