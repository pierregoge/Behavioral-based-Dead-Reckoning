function plot_var_etho_3(var,var2,var3,etho,v,name,name2,name3)

%      plot_var_etho(var,etho,v)		  % var is a matrix of a variable, var
%      1 is a matrix of a behavior, v the version of ethogram
%      or
%      plot_var_etho(var,etho,v,name)	  % Name is used to plot the name
%      in the figure
%
%      Plot one variable with the behavior in shaded color in back ground
%
%		 Inputs:
%      Var is the variable to plot in function of time
%	   Etho is the 3xX ethogram matrix with behavior, time start, time end
%      v is the version the ethogram
%      name is the variable name to plot 
%
%
%     pierregogendeau@gmail.com
%     Last modified: 13/03/2022


pre = 'box';
names = {};

for i = 1:length(etho)
    box(i,1:4) =  [etho(i,2) etho(i,2) etho(i,3) etho(i,3)];  % Behavior limit to set up the color box_x limit
end

% Variable value to set the color box y limit
boxy(1,1) = min(var(:,1))*1.4; 
boxy(1,4) = min(var(:,1))*1.4;
boxy(1,2) = max(var(:,1))*1.4; 
boxy(1,3) = max(var(:,1))*1.4;

% Variable value to set the color box y limit
boxy2(1,1) = min(var2(:,1))*1.4; 
boxy2(1,4) = min(var2(:,1))*1.4;
boxy2(1,2) = max(var2(:,1))*1.4; 
boxy2(1,3) = max(var2(:,1))*1.4;

% Variable value to set the color box y limit
boxy3(1,1) = min(var3(:,1))*1.4; 
boxy3(1,4) = min(var3(:,1))*1.4;
boxy3(1,2) = max(var3(:,1))*1.4; 
boxy3(1,3) = max(var3(:,1))*1.4;


figure

tiledlayout(3,1)

% First plot
ax1 = nexttile;
title('Variable plot with colored ethogram')
plot(var,'LineWidth',2);


xlabel('Sample number')
if nargin > 3
    ylabel(name)  % Plot variable if given as input
end
ylim([boxy(1,1) boxy(1,2)]);
xlim([1 length(var(:,1))])

for i=1:length(etho(:,1))
    box_color(i,1:3) = c_behavior(etho(i,1),v);    % Apply color box function. Depend of the ethogram version
    patch( box(i,1:4),boxy,box_color(i,1:3) ,'FaceAlpha',0.2) % Apply color patch with the defined behavior boxes
end


ax2 = nexttile;
plot(var2,'LineWidth',2);

xlabel('Sample number')
if nargin > 3
    ylabel(name2)  % Plot variable if given as input
end
ylim([boxy2(1,1) boxy2(1,2)]);
xlim([1 length(var2(:,1))])

for i=1:length(etho(:,1))
    box_color(i,1:3) = c_behavior(etho(i,1),v);    % Apply color box function. Depend of the ethogram version
    patch( box(i,1:4),boxy2,box_color(i,1:3) ,'FaceAlpha',0.2) % Apply color patch with the defined behavior boxes
end


ax3 = nexttile;
plot(var3,'LineWidth',2);

xlabel('Sample number')
if nargin > 3
    ylabel(name3)  % Plot variable if given as input
end
ylim([boxy3(1,1) boxy3(1,2)]);
xlim([1 length(var3(:,1))])

for i=1:length(etho(:,1))
    box_color(i,1:3) = c_behavior(etho(i,1),v);    % Apply color box function. Depend of the ethogram version
    patch( box(i,1:4),boxy3,box_color(i,1:3) ,'FaceAlpha',0.2) % Apply color patch with the defined behavior boxes
end


linkaxes([ax1 ax2, ax3],'x')

end

