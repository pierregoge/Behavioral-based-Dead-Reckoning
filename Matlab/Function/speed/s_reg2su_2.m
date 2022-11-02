function [speed_up,speed_depth] = s_reg2su_2(pitch,depth,Ad,fs,reg)


% Calculate horizontal speed with mean pitch and different pitch between the length of
% the input variables. Apply a linear regression to this value in function
% of the length, mean depth defined for the UP behavior. The linear
% regression include 4 variables : Speed_depth, mean pitch, length of
% behavior, mean depth.
%
%
%     Input :
%     -pitch        Pitch en rad
%	  -depth        Depth in meter
%
%     Output:
%     -speed_up   Speed during the UP phase in meter after regression
%     -speed_depth  Speed calculate with mean pitch and difference of depth
%
%     pierregogendeau@gmail.com
%     Last modified: 10/03/22

if nargin > 4 
    coeff(:,1) = reg.LinearModel.Coefficients(:,1);
else
    %Regression coefficients are calculated with the script : X.m
    %Model : load('trainedModel_ligth.mat')  -> trainedModel_up
    coeff = [0.583052822451819;0.0533062916015396;-0.0672008059924474;-0.0718818656796174];
end


[nb_str_x, avg_str_x] = stroke_nb(Ad);

len = length(pitch(:,1));

len_div = floor(len/5);

for i=1:5

lim_inf = (i-1)*len_div;
lim_sup = ((i)*len_div)-1;

if i==1
    lim_inf = 1;
end
if i ==4
    lim_sup = len;
end
    
m_pitch  = mean(pitch(lim_inf:lim_sup));   %Mean pitch
m_depth  = mean(depth(lim_inf:lim_sup));   %Mean pitch
diff_depth = depth(lim_sup,1)-depth(lim_inf,1);
m_pitch_deg = m_pitch*180/pi;

%Calculate horizontal displacement with difference of depth and mean pitch
disp_depth(lim_inf:lim_sup) = (diff_depth/(tan(m_pitch)));
speed_depth(lim_inf:lim_sup) = abs((disp_depth(lim_inf:lim_sup)/(len/fs)));  %Convert displacement in m to speed in m/s

end

m_pitch = m_pitch*180/pi; %rad to degree

%Apply regression specific for DOWN phases
speed_up = coeff(1,1) + coeff(2,1)*speed_depth + coeff(3,1)*speed_depth*nb_str_x ;
%speed_up = abs(disp_depth/(len/fs));  %Convert displacement in m to speed in m/s

end
