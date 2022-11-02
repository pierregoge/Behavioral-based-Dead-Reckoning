function [seq] = f_tt2seq_raw_ref(data_ref,fs)

%Format the input data reference from a timetables and store is
%to a var structure
%
%Input :  
%-data_ref is a timetables with displacement x,
%y, z, speed, lat, long
%-fs is the sampling frequency of the data in Hz
%
%Output :
%-var_ref is a structure with the tag data 
%
%     pierregogendeau@gmail.com
%     Last modified: 10/03/22


%Example of accelerometer format data  
%                   data: [1172115×3 double]
%               sampling: 'regular'
%          sampling_rate: 32
%     sampling_rate_unit: 'Hz'
%                  depid: 'mn12_186a'
%          creation_date: '24-Jul-2017 12:40:14'
%                history: 'read_ll3m'
%                   name: 'A'
%              full_name: 'Acceleration'
%            description: 'triaxial acceleration'
%                   unit: '1'
%              unit_name: 'counts'
%             unit_label: 'counts'
%            column_name: 'x,y,z'
%                  frame: 'tag'
%                   axes: 'FRU'
%                  files: '20120704-57537-Mn-XXXX-U-NoName-Acceleration…'

len = length(data_ref(:,1));

for i=1:len
%Displacement on X axis on earth frame
X.data = data_ref(i,1).X;
X.sampling = 'intermitent';
X.sampling_rate = fs;
X.sampling_rate_unit = 'Hz';
X.name = 'X';
X.full_name = 'Displacement on X';
X.unit = 'm';
X.frame = 'earth';

%Displacement on Y axis on earth frame
Y.data = data_ref(i,1).Y;
Y.sampling = 'intermitent';
Y.sampling_rate = fs;
Y.sampling_rate_unit = 'Hz';
Y.name = 'Y';
Y.full_name = 'Displacement on Y';
Y.unit = 'm';
Y.frame = 'earth';

%Turtle speed on earth frame
LAT.data = data_ref(i,1).LAT;
LAT.sampling = 'intermitent';
LAT.sampling_rate = fs;
LAT.sampling_rate_unit = 'Hz';
LAT.name = 'LAT';
LAT.full_name = 'Latitude';
LAT.unit = '°';
LAT.frame = 'earth';


%Turtle speed (10Hz) on earth frame
LON.data = data_ref(i,1).LON;
LON.sampling = 'intermitent';
LON.sampling_rate = fs;
LON.sampling_rate_unit = 'Hz';
LON.name = 'LON';
LON.full_name = 'Longitude';
LON.unit = '°';
LON.frame = 'earth';

var.X = X;
var.Y = Y;
var.LAT = LAT;
var.LON = LON;
seq(i,1).var = var;
seq(i,1).tstart = data_ref(i,1).lim_inf;
seq(i,1).tend = data_ref(i,1).lim_sup;
seq(i,1).t_unit = data_ref(i,1).time;
seq(i,1).fs = X.sampling_rate;
seq(i,1).fs_unit = X.sampling_rate_unit;
seq(i,1).type = 'seq_raw_ref';

end

end

