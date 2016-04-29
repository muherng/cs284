%% Code setup (Do not modify, but please read) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacles. Each obstacle is a cell in Obs. An obstacle is
% represented as a convex hull of a number of points. These points are
% stored in the cells of Obs.
% First row is x, second is y (position of vertices)
clear;
w = 0.5;
Obs{1} = [0 0;5 0;5 w;0 w]';
Obs{2} = [0 0;2*w 0;w 10;0 10]';
Obs{3} = [0 10-w;5 10;5 10+w;0 10+w]';
Obs{4} = [5-w 0;5+w 0;5+w 5;5 5]';
Obs{5} = [5-w 10+w;5+w 10+w;5+w 6.25;5 6.25]';
Obs{6} = [4 5;5+w 5;5+w 5+w;4 5+w]';
Obs{7} = [4 6.25;5+w 6.25;5+w 6.25+w;4 6.25+w]';

% Bounds on world
world_bounds_x = [-8,10];
world_bounds_y = [-4,14];


% Draw obstacles
figure(1); clf; hold on;
axis([world_bounds_x world_bounds_y]);

for k = 1:length(Obs)
    patch(Obs{k}(1,:),Obs{k}(2,:),'r');
end

% Start and goal positions
xy_start = [4;1]; plot(xy_start(1),xy_start(2),'bo','MarkerFaceColor','b','MarkerSize',10);
xy_goal = [-4;6]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10);

% Initialize RRT. The RRT will be represented as a 2 x N list of points. So
% each column represents a vertex of the tree.
rrt_verts = zeros(2,1000); 
rrt_verts(:,1) = xy_start;
rrt_distance = zeros(1,1000);
rrt_distance(1) = 0;
rrt_tree = zeros(1,1000);
rrt_tree(1) = 1;

N = 1;
nearGoal = false; % This will be set to true if goal has been reached
minDistGoal = 0.25; % This is the convergence criterion. We will declare
                    % success when the tree reaches within 0.25 in distance
                    % from the goal. DO NOT MODIFY.

% Extension parameter
d = 0.5; % This controls how far the RRT extends in each step. DO NOT MODIFY.
%d = 2.0
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% RRT algorithm
iter = 0;
found = 0;
while ~nearGoal
   % Sample point
    rnd = rand(1);
    % With probability 0.05, sample the goal. This promotes movement to the
    % goal.
    if found == 0
        if rnd < 0.05
            xy = xy_goal;
        else
            %% FILL ME IN
            % Sample (uniformly) from space (with probability 0.95). The space is defined
            % with the bounds world_bounds_x and world_bounds_y defined above.
            % So, the x coordinate should be sampled in the interval
            % world_bounds_x and the y coordinate from world_bounds_y.
            xy = [0;0]; % Should be a 2 x 1 vector
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            xy(1) = (world_bounds_x(2) - world_bounds_x(1))*rand() + world_bounds_x(1);
            xy(2) = (world_bounds_x(2) - world_bounds_x(1))*rand() + world_bounds_y(1);
        end
    else
        xy = [0;0]; % Should be a 2 x 1 vector
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        xy(1) = (world_bounds_x(2) - world_bounds_x(1))*rand() + world_bounds_x(1);
        xy(2) = (world_bounds_x(2) - world_bounds_x(1))*rand() + world_bounds_y(1);
    end
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Check if sample is collision free
    collFree = isCollisionFree(Obs,xy); % Write this function. 
                                        % Your code from part (a) will be useful here.
    % collFree = 1;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                        
                                        
    % If it's not collision free, continue with loop
    if ~collFree
        continue;
    end
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If it is collision free, find closest point in existing tree. 
    % The points in the existing tree are rrt_verts(:,1:N)
    [closest_vert,index] = closestVertex(rrt_verts(:,1:N),xy); % Write this function  
    % Extend tree towards xy from closest_vert. Use the extension parameter
    % d defined above as your step size. In other words, the Euclidean
    % distance between new_vert and closest_vert should be d (do not modify
    % d. It should be 0.5).
    if norm(xy.' - closest_vert) < d
        new_vert = xy.';
    else 
        new_vert = d*(xy.' - closest_vert)/norm(xy.' - closest_vert) + closest_vert;
    end
    % Check if new_vert is collision free
    collFree = isCollisionFree(Obs,new_vert); % Same function you wrote before.
    %collFree = 1;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    % If it is not collision free, continue with loop
     if ~collFree
        continue;
     end
     
%      
    
    %% DO NOT MODIFY THIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If it is collision free, add it to tree    
    N = N+1;
    if N > size(rrt_verts,2)
        rrt_verts = [rrt_verts zeros(size(rrt_verts))];
        rrt_distance = [rrt_distance zeros(size(rrt_distance))];
        rrt_tree = [rrt_tree zeros(size(rrt_tree))];
    end
    rrt_verts(:,N) = new_vert;
    rrt_distance(N) = rrt_distance(index) + norm(new_vert - closest_vert);
    rrt_tree(N) = index;
    
    [rrt_distance, rrt_tree] = rewire(rrt_verts, rrt_distance, rrt_tree, new_vert,N);
    
    
    % Plot extension (Comment the next 3 lines out if you want your code to
    % run a bit quicker. The plotting is useful for debugging though.)
    %plot(new_vert(1),new_vert(2),'bo','MarkerFaceColor','b','MarkerSize',5);
    %line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
    
    
    % Check if we have reached goal
    if norm(xy_goal.' - new_vert) < minDistGoal
        found = 1;
        goal_vert = N;
        disp('FOUND GOAL')
        disp(N);
    end
    
    if N == 1000
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
        path = reverse_path;
        disp(path);
        break;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
       
end

% Plot vertices in RRT
figure(1)
plot(rrt_verts(1,:),rrt_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);
for i = 1:size(path,2)-1
%for i = 1:N
    %disp(i);
    %line([rrt_verts(1,rrt_tree(i)),rrt_verts(1,i)],[rrt_verts(2,rrt_tree(i)),rrt_verts(2,i)]);
    line([path(1,i),path(1,i + 1)],[path(2,i),path(2,i+1)]);
end