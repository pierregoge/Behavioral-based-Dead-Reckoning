function [A,M,G,P] = f_tt2data(data_tag,fs)

%Format the input data from a timetables or a strcut to the datatag wiki
%recommend format (http://www.animaltags.org/doku.php?id=tagwiki:tools:formats)

%Input :  
%-data_tag is a timetables or struct times series with accelerometer,
%magnetometer, gyroscope and depth data corrected
%-fs is the sampling frequency of the data in Hz

%Output :
%-A is the accemerometer formated data 
%-M is the magnetometer formated data
%-G is the gyroscope formated data
%-P is the depth formated data
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

%Accel
A.data = data_tag.accel;
A.sampling = 'regular';
A.sampling_rate = fs;
A.sampling_rate_unit = 'Hz';
A.name = 'A';
A.full_name = 'Acceleration';
A.unit = 'm/s²';
A.frame = 'tag';

%Mag 
M.data = data_tag.mag;
M.sampling = 'regular';
M.sampling_rate = fs;
M.sampling_rate_unit = 'Hz';
M.name = 'M';
M.full_name = 'Magnetometer';
M.unit = 'uT';
M.frame = 'tag';

%Mag 
G.data = data_tag.gyro;
G.sampling = 'regular';
G.sampling_rate = fs;
G.sampling_rate_unit = 'Hz';
G.name = 'M';
G.full_name = 'Magnetometer';
G.unit = 'rad/s';
G.frame = 'tag';

%Depth
P.data = data_tag.depth;
P.sampling = 'regular';
P.sampling_rate = fs;
P.sampling_rate_unit = 'Hz';
P.name = 'M';
P.full_name = 'Depth';
P.unit = 'm';

end
