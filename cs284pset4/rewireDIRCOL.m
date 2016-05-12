function rrt_tree = rewireDIRCOL(rrt_tree,N,Q,R,T,explore)
%nv = new_vert.';
nv = rrt_tree(N).vertex;
cost_to_vert = rrt_tree(N).cost;
gamma = 10.0;
neighbor_radius = gamma*(log(N)/N)^0.5;
neighbor_radius = gamma;
[neighbors,policy_matrix,cost_matrix] = neighborsDIRCOL(rrt_tree,N,neighbor_radius,Q,R);
%disp(neighbors);
for i = 1:size(neighbors,2)
    vert = rrt_tree(neighbors(i)).vertex;
    %x_bar = vert - nv.';
    %cost_to_go = x_bar.'*cost_matrix(:,:,i)*x_bar;
    %[approx_vert,cost_to_go,iter_time] = steerLQR(nv,vert,policy_matrix(:,i),cost_matrix(:,:,i),Q,R);
    [approx_vert,action,time,cost_to_go] = steerDIRCOL(nv,vert,policy_matrix(:,i).',cost_matrix(:,:,i),Q,R,T,explore);
    if cost_to_go < 0.001
        continue;
    end
    new_cost = cost_to_vert + cost_to_go;
    if new_cost < rrt_tree(neighbors(i)).cost;
        disp('rewire');
        disp(rrt_tree(neighbors(i)).cost);
        disp(new_cost);
        old_parent = rrt_tree(neighbors(i)).parent;
        %decrement = rrt_tree(neighbors(i)).cost - new_cost;
        %root = neighbors(i);
        rrt_tree(neighbors(i)).cost = new_cost;
        rrt_tree(neighbors(i)).action = action;
        rrt_tree(neighbors(i)).time = time;
        rrt_tree(neighbors(i)).vertex = approx_vert;
        rrt_tree(neighbors(i)).parent = N;
        if rrt_tree(N).children == -1
            rrt_tree(N).children = neighbors(i);
        else
            rrt_tree(N).children = [rrt_tree(N).children,neighbors(i)];
        end
        overcomplete_children = rrt_tree(old_parent).children;
        rrt_tree(old_parent).children = overcomplete_children(overcomplete_children~=neighbors(i));
        %first_call = 1;
        %rrt_tree = deep_enforce_dynamics(rrt_tree, root, decrement,first_call);
        %rrt_distance(neighbors(i)) =  dist;
    end
end

return;

