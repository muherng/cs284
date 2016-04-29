function[rrt_distance, rrt_tree] = rewire(rrt_verts,rrt_distance,rrt_tree,new_vert,N)
nv = new_vert.';
dist_to_vert = rrt_distance(N);
%N = size(rrt_verts,2);
neighbors = zeros(1,10);
neighbor_radius = 1.5;
n = 0;
for i = 1:N-1
    if norm(rrt_verts(:,i) - nv) < neighbor_radius;
        n = n + 1;
        if n > size(neighbors)
           neighbors = [neighbors zeros(size(neighbors))];
        end
        neighbors(n) = i;
    end
end 
neighbors = neighbors(1:n);
%disp(neighbors);

for i = 1:size(neighbors,2)
    vert = rrt_verts(:,neighbors(i));
    dist = dist_to_vert + norm(nv - vert);
    if dist < rrt_distance(neighbors(i));
        %disp('REWIRING STEP');
        collfree = isCollisionFree(Obs,new_vert);
        if collfree
            rrt_distance(neighbors(i)) =  dist;
            rrt_tree(neighbors(i)) = N;
        end
    end
end

return;