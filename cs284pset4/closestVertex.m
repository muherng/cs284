function[vertex,index] = closestVertex(rrt_verts,xy)
dim = size(rrt_verts);
N = dim(2);
vertex = [0,0];
min_distance = 0;
index = 1;
for i = 1:N
    if i == 1
        min_distance = norm(rrt_verts(:,i) - xy);
        vertex = rrt_verts(:,i).';
        index = i;
    else
        if min_distance > norm(rrt_verts(:,i) - xy)
            min_distance = norm(rrt_verts(:,i) - xy);
            vertex = rrt_verts(:,i).';
            index = i;
        end
    end
end

return;