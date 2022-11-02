function [var] = f_tt2var_ref(data_ref,fs)

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

len = length(data_ref(1,:));

for i=1:len
%Displacement on X axis on earth frame
X.data = data_ref(1,i).X;
X.sampling = 'regular';
X.sampling_rate = fs;
X.sampling_rate_unit = 'Hz';
X.name = 'X';
X.full_name = 'Displacement on X';
X.unit = 'm';
X.frame = 'earth';

%Displacement on Y axis on earth frame
Y.data = data_ref(1,i).Y;
Y.sampling = 'regular';
Y.sampling_rate = fs;
Y.sampling_rate_unit = 'Hz';
Y.name = 'Y';
Y.full_name = 'Displacement on Y';
Y.unit = 'm';
Y.frame = 'earth';

%Turtle speed on earth frame
S.data = data_ref(1,i).S;
S.sampling = 'regular';
S.sampling_rate = fs;
S.sampling_rate_unit = 'Hz';
S.name = 'S';
S.full_name = 'Speed';
S.unit = 'm/s';
S.frame = 'earth';


%Turtle speed (10Hz) on earth frame
S10.data = data_ref(1,i).S10;
S10.sampling = 'regular';
S10.sampling_rate = fs*10;
S10.sampling_rate_unit = 'Hz';
S10.name = 'S10';
S10.full_name = 'Speed';
S10.unit = 'm/s';
S10.frame = 'earth';

var(i,1).X = X;s
var(i,1).Y = Y;
var(i,1).S = S;
var(i,1).S10 = S10;
var(i,1).tstart = data_ref(1,i).lim_inf;
var(i,1).tend = data_ref(1,i).lim_sup;
var(i,1).time = data_ref(1,i).time;

end

end

