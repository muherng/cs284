function [closest_vert,K_final,S_final,index] = closestVertexLQR(rrt_verts,xy)
dimensions = size(rrt_verts);
N = dimensions(2);
minimum = 0;
closest_vert = [0;0];
Q = eye(2);
R = 0.1; 
K_final = eye(2);
S_final = eye(2);
A = [0 1; -9.8*cos(xy(1)) -0.1];
B = [0;1];
[K,S] = lqr(A,B,Q,R);
index = 1;
for i = 1:N
    x_bar = rrt_verts(:,i) - xy;
    angle = rrt_verts(1,i) - xy(1);
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
    if i == 1
        minimum = cost_to_go;
        K_final = K;
        S_final = S;
        closest_vert = rrt_verts(:,i);
        index = i;
    else
        if cost_to_go < minimum
            minimum = cost_to_go;
            K_final = K;
            S_final = S;
            closest_vert = rrt_verts(:,i);
            index = i;
        end
    end
end
return;

