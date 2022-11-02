function [speed_down,speed_depth] = s_reg2sd(pitch,depth,fs,reg)


% Calculate speed with mean pitch and different pitch between the length of
% the input variables. Apply a linear regression to this value in function
% of the length, mean depth defined for the DOWN behavior. The linear
% regression include 4 variables : Speed_depth, mean pitch, length of
% behavior, mean depth.
%
%     Input :
%     -pitch        Pitch en rad
%	  -depth        Depth in meter
%
%     Output:
%     -speed_down   Speed during the DOWN phase in meter after regression
%     -speed_depth  Speed calculate with mean pitch and difference of depth
%
%     pierregogendeau@gmail.com
%     Last modified: 10/03/22

if nargin > 3 
    coeff(:,1) = reg.LinearModel.Coefficients(:,1);
else    
%Regression coefficients are calculated with the script : X.m 
%Model : load('trainedModel_light')  -> trainedModel_down
coeff = [0.106742257510672;-0.682599367854480];  
end

len = length(pitch(:,1));

%Difference of depth
diff_depth = depth(len,1)-depth(1,1);

%Calculate the mean pitch and mean depth
sum_pitch = 0;
sum_depth = 0;
for i=1:len 
    sum_pitch = sum_pitch + pitch(i,1);  %Sum of all the pitch
    sum_depth = sum_depth + depth(i,1);  %Sum of all the depth
end
 m_pitch  = sum_pitch/len;   %Mean pitch
 m_depth  = sum_depth/len;   %Mean pitch

%Calculate horizontal displacement with difference of depth and mean pitch
disp_depth = (diff_depth/(tan(m_pitch)));
speed_depth = abs((disp_depth/(len/fs)));  %Convert displacement in m to speed in m/s

m_pitch = m_pitch*180/pi; %rad to degree

%Apply regression specific for DOWN phases
speed_down =-1*( coeff(1,1) + coeff(2,1)*speed_depth);
%speed_down = abs(disp_depth/(len/fs));  %Convert displacement in m to speed in m/s

end

