function rrt_tree = deep_enforce_dynamics(rrt_tree, root, decrement,first_call)
if rrt_tree(root).children == -1
    %disp(root);
    return;
end
if first_call
    %disp(root);
    children = rrt_tree(root).children;
    for i = 1:size(children)
        rrt_tree = deep_enforce_dynamics(rrt_tree,children(i),decrement,0);
    end
else
    %disp(root);
    rrt_tree(root).cost = rrt_tree(root).cost - decrement;
    start = rrt_tree(rrt_tree(root).parent).vertex;
    action = rrt_tree(root).action;
    time = rrt_tree(root).time;
    [approx_vert,~] = forward_simulate(start,action,time);
    rrt_tree(root).vertex = approx_vert;
    children = rrt_tree(root).children;
    for i = 1:size(children)
        rrt_tree = deep_enforce_dynamics(rrt_tree,children(i),decrement,0);
    end
end
end

