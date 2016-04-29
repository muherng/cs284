function [dy] = pendulumLQR(t,y,weird,K,xy)
x_bar = y - xy;
angle = y(1) - xy(1);
if angle < 0
    complement = 2*pi + angle;
else
    complement = 2*pi - angle;
end
if angle < abs(complement)
    x_bar(1) = angle;
else
    x_bar(1) = complement;
end
u = -K*x_bar;
if u > 5
    u = 5;
end
if u < -5
    u = -5;
end
dy = zeros(2,1);
dy(1) = y(2);
dy(2) = u -9.8*sin(y(1)) - 0.1*y(2);
end

