function rrt_tree = rewireLQR(rrt_tree,N,Q,R,T,active_nodes,explore)
%nv = new_vert.';
nv = rrt_tree(N).vertex;
cost_to_vert = rrt_tree(N).cost;
%200 for Q identity, R = 1
%
gamma = 50.0;
%neighbor_radius = gamma*(log(active_nodes)/active_nodes)^0.5;
neighbor_radius = gamma;
[neighbors,policy_matrix,cost_matrix] = neighborsLQR(rrt_tree,N,neighbor_radius,Q,R);
%disp(neighbors);
rewire_count = 0;
%disp(size(neighbors,2));
for i = 1:size(neighbors,2)
    vert = rrt_tree(neighbors(i)).vertex;
    %x_bar = vert - nv.';
    %cost_to_go = x_bar.'*cost_matrix(:,:,i)*x_bar;
    %[approx_vert,cost_to_go,iter_time] = steerLQR(nv,vert,policy_matrix(:,i),cost_matrix(:,:,i),Q,R);
    [approx_vert,action,time,cost_to_go] = steerLQR(nv,vert,policy_matrix(:,i).',cost_matrix(:,:,i),Q,R,T,explore);
    if cost_to_go < 0.001
        continue;
    end
    new_cost = cost_to_vert + cost_to_go;
    if new_cost < rrt_tree(neighbors(i)).cost
        rewire_count = rewire_count + 1;
        %disp('rewire');
        %disp(rrt_tree(neighbors(i)).cost);
        %disp(new_cost);
        old_parent = rrt_tree(neighbors(i)).parent;
        decrement = rrt_tree(neighbors(i)).cost - new_cost;
        root = neighbors(i);
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
        first_call = 1;
        rrt_tree = deep_enforce_dynamics(rrt_tree, root, decrement,first_call);
        %rrt_distance(neighbors(i)) =  dist;
    end
end
disp(rewire_count);
return;
