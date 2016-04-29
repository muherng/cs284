function [new_vert] = extendEuclidean(closest_vert,xy)
new_vert = [0;0];
minimum = 0;
for i = 1:20
    u = -5 + i*0.5;
    
    [t,y] = ode45('pendulumcats',[0 0.1], closest_vert,'mysterious', u);
    dimensions = size(y);
    end_point = y(dimensions(1),:).';
    if end_point(1) < - pi/2
        end_point(1) = 3*pi/2 + (end_point(1) + pi/2);
    end
    if end_point(1) > 3*pi/2
        end_point(1) = -pi/2 + end_point(1) - 3*pi/2;
    end
    version1 = end_point - xy;
    version2 = version1;
    version2(1) = 2*pi - version1(1);
    distance = min(norm(version1),norm(version2));
    if i == 1
        minimum = distance;
        new_vert = end_point;
    else
        if distance < minimum
            minimum = distance;
            new_vert = end_point;
        end
    end
end
return;