function [collision_free] = isCollisionFree(Obs,xy)
obs_size = 7;
in = 1;
for i = 1:obs_size
    quad = Obs{i};
    %ch = convhull(quad(1,:),quad(2,:));
    sides = 4;
    polygon = quad;
    %for j = 1:4
    %    polygon(:,j) = quad(:,ch(j));
    %end
    in = in && (~inpolygon(xy(1),xy(2),polygon(1,:),polygon(2,:)));
end

collision_free = in;

return;

