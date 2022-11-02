function h = circle(x,y,r,color,name,width)

th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit,color,'DisplayName',name,'LineWidth',width);

end