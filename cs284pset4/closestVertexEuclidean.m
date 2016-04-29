function [vert,index] = closestVertexEuclidean(rrt_verts,xy)

dimensions = size(rrt_verts);
N = dimensions(2);
minimum = 0;
vert = [0;0];
index = 1;
for i = 1:N
    version1 = rrt_verts(:,i) - xy;
    version2 = version1;
    version2(1) = 2*pi - version1(1);
    if i == 1
        minimum = min(norm(version1),norm(version2));
        vert = rrt_verts(:,i);
        index = i;
    else
        distance = min(norm(version1),norm(version2));
        if distance < minimum
            minimum = distance;
            vert = rrt_verts(:,i);
            index = i;
        end
    end
end

return; 