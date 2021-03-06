function path = reconstruct_path(rrt_tree,goal_vert)
reverse_path = rrt_tree(goal_vert).vertex;
child_node = goal_vert;
parent_node = rrt_tree(goal_vert).parent;
%n = 2;
while parent_node ~= 1
    [~,intermediate] = forward_simulate(rrt_tree(parent_node).vertex,rrt_tree(child_node).action,rrt_tree(child_node).time);
    reverse_path = [reverse_path fliplr(intermediate(:,1:size(intermediate,2)-1))];
    child_node = parent_node;
    parent_node = rrt_tree(parent_node).parent;
    if parent_node == 1
        [~,intermediate] = forward_simulate(rrt_tree(parent_node).vertex,rrt_tree(child_node).action,rrt_tree(child_node).time);
        reverse_path = [reverse_path fliplr(intermediate(:,1:size(intermediate,2)-1))];
        break;
    end
%    n = n + 1;
%     if n > size(reverse_path,2);
%         reverse_path = [reverse_path zeros(size(reverse_path))];
%     end
end

%reverse_path(:,n) = xy_start;
%reverse_path = reverse_path(:,1:n);
path = fliplr(reverse_path);
end

