function [neighbors,policy_matrix,cost_matrix] = neighborsLQR(rrt_tree,N,neighbor_radius,Q,R)
xy = rrt_tree(N).vertex;
neighbors = zeros(1,1000);
policy_matrix = zeros(2,1000);
cost_matrix = zeros(2,2,1000);
A = [0 1; -9.8*cos(xy(1)) -0.1];
B = [0;1];
[~,S] = lqr(A,B,Q,R);
index = 1;
for i = 1:N-1
    if rrt_tree(i).cost == -1
        continue
    end
    x_bar = rrt_tree(i).vertex - xy;
    %angle = rrt_verts(1,i) - xy(1);
    angle = x_bar(1);
    if angle < 0
        complement = 2*pi + angle;
    else 
        complement = 2*pi - angle;
    end
    if angle < abs(complement)
        x_bar(1) = angle;
    else
        x_bar(1) = complement;
    end
    cost_to_go = x_bar.' * S * x_bar;
    if cost_to_go < neighbor_radius
        if index > size(neighbors,2)
            neighbors = [neighbors zeros(size(neighbors))];
            policy_matrix = [policy_matrix zeros(size(policy_matrix))];
            cost_matrix = cat(3,cost_matrix,zeros(size(cost_matrix)));
        end
        A_vert = [0 1; -9.8*cos(rrt_tree(i).vertex(1)) -0.1];
        B_vert = [0;1];
        [K_vert,S_vert] = lqr(A_vert,B_vert,Q,R);
        policy_matrix(:,index) = K_vert;
        cost_matrix(:,:,index) = S_vert;
        neighbors(index) = i;
        index = index + 1;
    end
end

neighbors = neighbors(1:index-1);
policy_matrix = policy_matrix(:,1:index-1);
cost_matrix = cost_matrix(:,:,1:index-1);

return;