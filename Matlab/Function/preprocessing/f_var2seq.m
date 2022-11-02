function [seq] = f_var2seq(var,tstart,tend)

%Format the input from a strcut to the datatag wiki
%recommend format (http://www.animaltags.org/doku.php?id=tagwiki:tools:formats)
%
%Input :  
%-data_tag is a timetables or struct times series with accelerometer,
%magnetometer, gyroscope and depth data corrected
%-tstart is the sampling frequency of the data in Hz
%
%Output :
%-seq is the strcuture with the var structure, the sampling frequency of
%the fastest sensors, start and end time
%
%     pierregogendeau@gmail.com
%     Last modified: 10/03/22

if nargin<2
   tstart = 0;
   tend = length(var.A.data(:,1))/var.A.sampling_rate;
end
    
    
seq.var = var;
seq.tstart = tstart;
seq.tend = tend;
seq.t_unit = 'sec';
seq.fs_max = seq.var.A.sampling_rate;
seq.fs_unit = 'Hz';
seq.t_unit = 'sec';
seq.type = 'sequence';

end
