%% Simple math calcs
clear all;
close all;
clc;

for i = 1
    x0 = 0;
    x1 = (-4.61287 - (-4.61287)) + 4;
    y0 = 0;
    y1 = (1.50581 -  1.50581) +1;
    
    plot(x0,y0,'*');
    hold on
    plot(x1,y1,'*');
    hold on
    grid on
    
    % Finding Arc Radius Between Two Points
    b = x1-x0;
    a = y1 - y0;
    F = sqrt(b^2+a^2);
    rho = asin(b/F) - asin(a/F);
    r = -(a/(sin(rho)-1));
    
    % Finding Phi (Angle Between two points)
    phi = asin(b/r);
    
    % Finding Arc Length
    arc_length = r*phi;
    
    
    circle(x0, r, r,phi);
end

% Plotting the Arc
function [] = circle(x,y,r,phi)
hold on
%th = -(pi/2)+phi:-pi/250:-(pi/2);
th = 0:pi/250:2*pi;
th = th-(pi/2);
deg = th*(180/pi);

xunit = zeros(length(th));
yunit = zeros(length(th));

for i = 1:length(th)
    xunit(i) = r * cos(th(i)) + x;
    yunit(i) = r * sin(th(i)) + y;
end
plot(xunit,yunit)
hold on
end
