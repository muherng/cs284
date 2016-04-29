%% Code setup (Do not modify, but please read) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Bounds on world
world_bounds_th = [-pi/2,(3/2)*pi];
world_bounds_thdot = [-10,10];

% Start and goal positions
figure(1); clf;
xy_start = [0;0]; plot(xy_start(1),xy_start(2),'bo','MarkerFaceColor','b','MarkerSize',10);
xy_goal = [pi;0]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10); drawnow;

% Initialize RRT. The RRT will be represented as a 2 x N list of points. So
% each column represents a vertex of the tree.
rrt_verts = zeros(2,1000);
rrt_verts(:,1) = xy_start;
rrt_distance = zeros(1,1000);
rrt_distance(1) = 0;
rrt_tree = zeros(1,1000);
rrt_tree(1) = 1;
rrt_child = cell(1,1000);

N = 1;
nearGoal = false; % This will be set to true if goal has been reached
minDistGoal = 0.25; % This is the convergence criterion. We will declare
                    % success when the tree reaches within 0.25 in distance
                    % from the goal. DO NOT MODIFY.

path = zeros(1,1000);
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Choose one of these methods
% method = 'euclidean'; % Euclidean distance metric (part b of problem)
 method = 'lqr'; % LQR distance metric (part d of problem)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure(1); hold on;
axis([world_bounds_th, world_bounds_thdot]);
 hxy = plot(0,0,'ro');
found = 0;
xy_debug = [0;0];
K_debug = [0;0];
% RRT algorithm
while ~nearGoal
    % Sample point
    rnd = rand(1);
    % With probability 0.05, sample the goal. This promotes movement to the
    % goal.
    if rnd < 0.05
        xy = xy_goal;
        xy_debug = [xy_debug xy];
    else
        % Sample from space with probability 0.95
        xs = (world_bounds_th(2) - world_bounds_th(1))*rand(1) + world_bounds_th(1);
        ys = (world_bounds_thdot(2) - world_bounds_thdot(1))*rand(1) + world_bounds_thdot(1);
        xy = [xs;ys];
        xy_debug = [xy_debug xy];
    end
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(method, 'euclidean')
        [closest_vert,index] = closestVertexEuclidean(rrt_verts(:,1:N),xy); % Write this function
    elseif strcmp(method, 'lqr')
        [closest_vert,K,S,index] = closestVertexLQR(rrt_verts(:,1:N),xy); % Write this function
        K_debug = [K_debug K.'];
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if strcmp(method, 'euclidean')
        %disp('extending euclidean')
        new_vert = extendEuclidean(closest_vert,xy); % Write this function
    else
        new_vert = extendLQR(closest_vert,xy,K); % Write this function
    end
        
    delete(hxy);
    figure(1);
    hxy = plot(xy(1),xy(2),'r.');axis([world_bounds_th, world_bounds_thdot]); 

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
     
    % Plot extension (Comment the next few lines out if you want your code to
    % run a bit quicker. The plotting is useful for debugging though.)
    %figure(1)
    %hold on
    %plot(new_vert(1),new_vert(2),'bo','MarkerFaceColor','b','MarkerSize',5);
    % Plot line (but only if we are not wrapping to the other side of the
    % plot)
%     if abs(closest_vert(1) - new_vert(1)) < 0.75*(2*pi)
%         line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
%     end
    %axis([world_bounds_th, world_bounds_thdot]);

    
    %% DO NOT MODIFY THIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If it is collision free, add it to tree  
    N = N+1;
    if N > size(rrt_verts,2)
        rrt_verts = [rrt_verts zeros(size(rrt_verts))];
        rrt_distance = [rrt_distance zeros(size(rrt_distance))];
        rrt_tree = [rrt_tree zeros(size(rrt_tree))];
        rrt_child = [rrt_child cell(1,size(rrt_child,2))];
    end
    rrt_verts(:,N) = new_vert;
    rrt_distance(N) = rrt_distance(index) + (new_vert - closest_vert).'*S*(new_vert - closest_vert);
    rrt_tree(N) = index;
    rrt_child{index} = [rrt_child{index},N];
    [rrt_distance, rrt_tree,rrt_child] = rewireLQR(rrt_verts, rrt_distance, rrt_tree,rrt_child, new_vert,N);
    %disp(N);
    % Check if we have reached goal
    if norm(xy_goal-new_vert) < minDistGoal
        if found == 0
            goal_vert = N;
            current_cost = rrt_distance(goal_vert);
            disp('FOUND GOAL')
            disp(N);
            disp('descending cost');
            disp(current_cost);
        else
            if rrt_distance(N) < rrt_distance(goal_vert)
                goal_vert = N;
                current_cost = rrt_distance(goal_vert);
                disp('FOUND GOAL')
                disp(N);
                disp('descending cost');
                disp(current_cost);
            end
        end
        found = 1;
    end
    
    if found == 1
%         if mod(N,10) == 0
%             disp('current distance to goal');
%             disp(rrt_distance(goal_vert));
%             disp(N);
%         end
        if rrt_distance(goal_vert) < current_cost
            current_cost = rrt_distance(goal_vert);
            disp('descending cost');
            disp(current_cost);
            disp(N);
        end
    end
    
    if N == 1000
        if found
            %disp(N);
            %disp(rrt_distance(N));
            reverse_path = zeros(2,1000);
            reverse_path(:,1) = [rrt_verts(:,goal_vert)];
            back_iter = rrt_tree(goal_vert);
            n = 2;
            while back_iter ~= 1
                reverse_path(:,n) = rrt_verts(:,back_iter);
                back_iter = rrt_tree(back_iter);
                n = n + 1;
                if n > size(reverse_path,2);
                    reverse_path = [reverse_path zeros(size(reverse_path))];
                end
            end
            
            reverse_path(:,n) = xy_start;
            reverse_path = reverse_path(:,1:n);
            path = fliplr(reverse_path);
            %disp(path);
        end
        break;
    end
    
    
%     if norm(xy_goal-new_vert) < minDistGoal
%         iter = 2;
%         reverse_path = zeros(2,N);
%         reverse_path(:,1) = new_vert;
%         previous = index;
%         if previous == 1
%             disp('NO');
%         end
%         
%         while previous ~= 1
%             reverse_path(:,iter) = rrt_verts(:,previous); 
%             previous = path(previous);
%             iter = iter + 1;
%             if previous == 1
%                 reverse_path(:,iter) = xy_start;
%             end
%         end
%         final_path = zeros(2,iter);
%         for i = 1:iter
%             final_path(:,i) = reverse_path(:,iter + 1 - i);
%         end
%         disp(final_path);
%         break;
%     end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
       
end

[true_path,true_cost] = verify_path(path);
% Plot vertices in RRT
%hold on;
%plot(rrt_verts(1,:),rrt_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);
% figure(2)
% xy_start = [0;0]; plot(xy_start(1),xy_start(2),'bo','MarkerFaceColor','b','MarkerSize',10);
% xy_goal = [pi;0]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10); drawnow;
% plot(final_path(1,:),final_path(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);

figure(1)
%plot(rrt_verts(1,:),rrt_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);
%for i = 1:size(path,2)-1
for i = 1:N
    %disp(i);
    if abs(rrt_verts(1,rrt_tree(i)) - rrt_verts(1,i)) < 0.75*(2*pi)
    %if abs(path(1,i) - path(1,i+1)) < 0.75*(2*pi)
        line([rrt_verts(1,rrt_tree(i)),rrt_verts(1,i)],[rrt_verts(2,rrt_tree(i)),rrt_verts(2,i)]);
        %line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
        %line([path(1,i),path(1,i + 1)],[path(2,i),path(2,i+1)]);
    end
    %line([rrt_verts(1,rrt_tree(i)),rrt_verts(1,i)],[rrt_verts(2,rrt_tree(i)),rrt_verts(2,i)]);
    %line([path(1,i),path(1,i + 1)],[path(2,i),path(2,i+1)]);
end

