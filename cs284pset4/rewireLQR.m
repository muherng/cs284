function[rrt_distance, rrt_tree,rrt_child] = rewireLQR(rrt_verts,rrt_distance,rrt_tree,rrt_child,new_vert,N)
nv = new_vert.';
dist_to_vert = rrt_distance(N);
neighbor_radius = 1.0;
rejection_radius = 0.01;
Q = eye(2);
R = 1.0;
%N = size(rrt_verts,2);
[neighbors,policy_matrix,cost_matrix] = neighborsLQR(rrt_verts,N,new_vert,neighbor_radius,Q,R);
%disp(neighbors);
for i = 1:size(neighbors,2)
    vert = rrt_verts(:,neighbors(i));
    %x_bar = vert - nv.';
    %cost_to_go = x_bar.'*cost_matrix(:,:,i)*x_bar;
    [approx_vert,cost_to_go,iter_time] = steerLQR(nv,vert,policy_matrix(:,i),cost_matrix(:,:,i),Q,R);
    dist = dist_to_vert + cost_to_go;
    sim_gap = approx_vert.' - vert;
    %d = norm(approx_vert.' - vert);
    cost_gap = sim_gap.'*cost_matrix(:,:,i)*sim_gap;  
    if  cost_gap < rejection_radius
        if dist < rrt_distance(neighbors(i));
            %disp('REWIRING STEP');
            decrease = rrt_distance(neighbors(i)) - dist;
            root = neighbors(i);
            false_parent = rrt_tree(neighbors(i));
            rrt_tree(neighbors(i)) = N;
            rrt_child{N} = [rrt_child{N},neighbors(i)];
            overcomplete_children = rrt_child{false_parent};
            rrt_child{false_parent} = overcomplete_children(overcomplete_children~=neighbors(i));
            rrt_distance = update_distance(rrt_child, rrt_distance, root, decrease);
            %rrt_distance(neighbors(i)) =  dist;
            
        end
    end
end

return;
