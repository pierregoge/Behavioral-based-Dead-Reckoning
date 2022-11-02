function plot_var_etho_1(var,etho,v,name)

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



plot(var,'black','LineWidth',2)

title('Variable plot with colored ethogram')
xlabel('Sample number')
if nargin > 3
    ylabel(name)  % Plot variable if given as input
end
ylim([boxy(1,1) boxy(1,2)]);
xlim([1 length(var(:,1))])


for i=1:length(etho(:,1))
    box_color(i,1:3) = c_behavior(etho(i,1),v);    % Apply color box function. Depend of the ethogram version
    patch( box(i,1:4),boxy,box_color(i,1:3),'FaceAlpha',0.2) % Apply color patch with the defined behavior boxes
    hold on
end



end

