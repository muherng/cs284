function [new_vert,action,time,cost] = extendLQR(closest_vert,xy,K)
[t,y] = ode45('pendulumLQR',[0 0.1], closest_vert,'mysterious', K, xy);
dimensions = size(y);
end_point = y(dimensions(1),:).';
if end_point(1) < - pi/2
    end_point(1) = 3*pi/2 + (end_point(1) + pi/2);
end
if end_point(1) > 3*pi/2
    end_point(1) = -pi/2 + end_point(1) - 3*pi/2;
end
new_vert = end_point;
return;