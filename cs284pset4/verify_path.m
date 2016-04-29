function [true_path,true_cost] = verify_path(path)
Q = eye(2);
R = 1.0;
true_cost = zeros(size(path));
new_vert = path(:,1);
true_path = zeros(size(path));
true_path(:,1) = new_vert;
for i = 1:size(path,2)-1
    A = [0 1; -9.8*cos(path(1,i)) -0.1];
    B = [0;1];
    [K,S] = lqr(A,B,Q,R);
    [new_vert,cost,iter_time] = steerLQR(new_vert,path(:,i+1),K.',S,Q,R);
    true_cost(:,i+1) = cost;
    true_path(:,i+1) = new_vert;
end
end

