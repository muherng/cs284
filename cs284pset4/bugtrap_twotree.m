%% Code setup (Do not modify, but please read) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Obstacles. Each obstacle is a cell in Obs. An obstacle is
% represented as a convex hull of a number of points. These points are
% stored in the cells of Obs.
% First row is x, second is y (position of vertices)
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

rrt_verts_goal = zeros(2,1000);
rrt_verts_goal(:,1) = xy_goal;

N = 1;
M = 1;
nearGoal = false; % This will be set to true if goal has been reached
minDistGoal = 0.25; % This is the convergence criterion. We will declare
                    % success when the tree reaches within 0.25 in distance
                    % from the goal. DO NOT MODIFY.

% Extension parameter
d = 0.5; % This controls how far the RRT extends in each step. DO NOT MODIFY.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
connected = 0;
% RRT algorithm
while ~nearGoal
   % Sample point
    rnd = rand(1);
    % With probability 0.05, sample the goal. This promotes movement to the
    % goal.
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
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Check if sample is collision free
    collFree = isCollisionFree(Obs,xy); % Write this function. 
                                        % Your code from part (a) will be useful here.
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                                        
                                        
    % If it's not collision free, continue with loop
    if ~collFree
        continue;
    end
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If it is collision free, find closest point in existing tree. 
    % The points in the existing tree are rrt_verts(:,1:N)
    closest_vert = closestVertex(rrt_verts(:,1:N),xy); % Write this function  
    % Extend tree towards xy from closest_vert. Use the extension parameter
    % d defined above as your step size. In other words, the Euclidean
    % distance between new_vert and closest_vert should be d (do not modify
    % d. It should be 0.5).
    new_vert = d*(xy.' - closest_vert)/norm(xy.' - closest_vert) + closest_vert;
    % Check if new_vert is collision free
    collFree = isCollisionFree(Obs,new_vert); % Same function you wrote before.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    % If it is not collision free, continue with loop
     if ~collFree
        continue;
     end
    
    N = N+1;
    if N > size(rrt_verts,2)
        rrt_verts = [rrt_verts zeros(size(rrt_verts))];
    end
    rrt_verts(:,N) = new_vert;
    figure(1)
    plot(new_vert(1),new_vert(2),'bo','MarkerFaceColor','b','MarkerSize',5);
    line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
     
    closest_vert_goal = closestVertex(rrt_verts_goal(:,1:N),new_vert.'); 
    if norm(new_vert - closest_vert_goal) < d
        new_vert_goal = new_vert;
        connected = 1;
    else
        new_vert_goal = d*(new_vert - closest_vert_goal)/norm(new_vert - closest_vert_goal) + closest_vert_goal;
    end
    collFree = isCollisionFree(Obs,new_vert_goal);
    
    if ~collFree
        copy = zeros(2,1000);
        copy(:,:) = rrt_verts(:,:);
        
        rrt_verts(:,:) = rrt_verts_goal(:,:);
        rrt_verts_goal(:,:) = copy(:,:);
        
        goal = xy_goal;
        xy_goal = xy_start;
        xy_start = goal;
        
        T = N;
        N = M;
        M = T;
        
        continue;
    end

     
    % Plot extension (Comment the next 3 lines out if you want your code to
    % run a bit quicker. The plotting is useful for debugging though.)
    
    %line([closest_vert_goal(1),new_vert_goal(1)],[closest_vert_goal(2),new_vert_goal(2)]);
%      
    
    %% DO NOT MODIFY THIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If it is collision free, add it to tree    
    M = M+1;
    if M > size(rrt_verts_goal,2)
        rrt_verts_goal = [rrt_verts_goal zeros(size(rrt_verts_goal))];
    end
    rrt_verts_goal(:,M) = new_vert_goal;
    plot(new_vert_goal(1),new_vert_goal(2),'bo','MarkerFaceColor','b','MarkerSize',5);
    line([closest_vert_goal(1),new_vert_goal(1)],[closest_vert_goal(2),new_vert_goal(2)]);
    % Check if we have reached goal
%     if norm(xy_goal.' - new_vert) < minDistGoal
%         break;
%     end
    
    if connected == 1
        disp(N);
        break;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
       
end

% Plot vertices in RRT
%plot(rrt_verts(1,:),rrt_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);
%plot(rrt_verts_goal(1,:),rrt_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);






