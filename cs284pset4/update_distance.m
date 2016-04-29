function [rrt_distance] = update_distance(rrt_child, rrt_distance, root, decrease)
rrt_distance(root) = rrt_distance(root) - decrease;
if size(rrt_child{root},2) > 0
    for i = 1:size(rrt_child{root},2)
        rrt_distance = update_distance(rrt_child,rrt_distance,rrt_child{root}(i),decrease);
    end
end
end

