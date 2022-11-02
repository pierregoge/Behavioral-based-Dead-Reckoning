
load('data_raw_1.mat')
load('data_raw_2.mat')
load('data_raw_3.mat')
load('data_raw_4.mat')
load('data_raw_5.mat')
load('data_raw_6.mat')


data_raw.depth = [data_raw_1.depth; data_raw_2.depth; data_raw_3.depth; data_raw_4.depth; data_raw_5.depth; data_raw_6.depth];

data_raw.accel = [data_raw_1.accel; data_raw_2.accel; data_raw_3.accel; data_raw_4.accel; data_raw_5.accel; data_raw_6.accel];

data_raw.mag = [data_raw_1.mag; data_raw_2.mag; data_raw_3.mag; data_raw_4.mag; data_raw_5.mag; data_raw_6.mag];

data_raw.gyro = [data_raw_1.gyro; data_raw_2.gyro; data_raw_3.gyro; data_raw_4.gyro; data_raw_5.gyro; data_raw_6.gyro];

data_raw.spin = [data_raw_1.spin; data_raw_2.spin; data_raw_3.spin; data_raw_4.spin; data_raw_5.spin; data_raw_6.spin];

data_raw.time = [data_raw_1.time; data_raw_2.time; data_raw_3.time; data_raw_4.time; data_raw_5.time; data_raw_6.time];
