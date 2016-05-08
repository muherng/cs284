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
rrt_tree = [];
for i = 1:1000
    field1 = 'parent';  value1 = -1;
    field2 = 'children';  value2 = [-1];
    field3 = 'action';  value3 = [0];
    field4 = 'cost';  value4 = -1;
    field5 = 'vertex'; value5 = [0;0];
    field6 = 'time'; value6 = [0];
    node = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6);
    rrt_tree = [rrt_tree node];
end

rrt_tree(1).parent = 1;
rrt_tree(1).cost = 0;
rrt_tree(1).vertex = xy_start;

%rrt_verts = zeros(2,1000);
%rrt_verts(:,1) = xy_start;
%rrt_distance = zeros(1,1000);
%rrt_distance(1) = 0;
%rrt_tree = zeros(1,1000);
%rrt_tree(1) = 1;
%rrt_child = cell(1,1000);
best_cost = [];
Q = eye(2);
R = 1.0;
T = 1.0;
N = 1;
explore = 1;
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
    if rnd < 0.1
        xy = xy_goal;
        %xy_debug = [xy_debug xy];
    else
        % Sample from space with probability 0.95
        xs = (world_bounds_th(2) - world_bounds_th(1))*rand(1) + world_bounds_th(1);
        ys = (world_bounds_thdot(2) - world_bounds_thdot(1))*rand(1) + world_bounds_thdot(1);
        xy = [xs;ys];
        %xy_debug = [xy_debug xy];
    end
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [K,S,index] = closestVertexLQR(rrt_tree(1:N),xy,Q,R); % Write this function
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% FILL ME IN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %we have set explore to 1 constantly 
    [new_vert,action,time,cost] = steerLQR(rrt_tree(index).vertex,xy,K,S,Q,R,T,explore); % Write this function
    if rrt_tree(index).children ~= -1
        for i = 1:size(rrt_tree(index).children,2)
            if norm(new_vert - rrt_tree(rrt_tree(index).children(i)).vertex) < 0.001
                %disp('Collision');
                continue;
            end
        end
    end
    delete(hxy);
    figure(1);
    hxy = plot(xy(1),xy(2),'r.');axis([world_bounds_th, world_bounds_thdot]);
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
     
    % Plot extension (Comment the next few lines out if you want your code to
    % run a bit quicker. The plotting is useful for debugging though.)
%     figure(1)
%     hold on
%     plot(new_vert(1),new_vert(2),'bo','MarkerFaceColor','b','MarkerSize',5);
%     %Plot line (but only if we are not wrapping to the other side of the
%     %plot)
%     if abs(rrt_tree(index).vertex(1) - new_vert(1)) < 0.75*(2*pi)
%         line([rrt_tree(index).vertex(1),new_vert(1)],[rrt_tree(index).vertex(2),new_vert(2)]);
%     end
%     axis([world_bounds_th, world_bounds_thdot]);

    
    %% DO NOT MODIFY THIS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % If it is collision free, add it to tree  
    N = N+1;
    if N > size(rrt_tree,2)
        field1 = 'parent';  value1 = -1;
        field2 = 'children';  value2 = [-1];
        field3 = 'action';  value3 = [0];
        field4 = 'cost';  value4 = -1;
        field5 = 'vertex'; value5 = [0;0];
        field6 = 'time'; value6 = [0];
        node = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6);
        rrt_tree = [rrt_tree node];
    end
    rrt_tree(N).vertex = new_vert;
    rrt_tree(N).action = action;
    rrt_tree(N).time = time;
    rrt_tree(N).cost = rrt_tree(index).cost + cost;
    rrt_tree(N).parent = index;
    if rrt_tree(index).children == -1
        rrt_tree(index).children = N;
    else
        rrt_tree(index).children = [rrt_tree(index).children N];
    end
    if ~explore
        %T = 0.05;
        rrt_tree = rewireLQR(rrt_tree,N,Q,R,T,explore);
    end
    
    if mod(N,10) == 0 && found == 1
        disp(N);
        disp(current_cost);
    end
    %disp(N);
    
    
    
%     figure(1)
%     %plot(rrt_verts(1,:),rrt_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);
%     %for i = 1:size(path,2)-1
%     for i = 1:N
%         %disp(i);
%         if abs(rrt_tree(i).vertex(1) - rrt_tree(rrt_tree(i).parent).vertex(1)) < 0.75*(2*pi)
%             %if abs(path(1,i) - path(1,i+1)) < 0.75*(2*pi)
%             line([rrt_tree(rrt_tree(i).parent).vertex(1),rrt_tree(i).vertex(1)],[rrt_tree(rrt_tree(i).parent).vertex(2),rrt_tree(i).vertex(2)]);
%             %line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
%             %line([path(1,i),path(1,i + 1)],[path(2,i),path(2,i+1)]);
%         end
%         %line([rrt_verts(1,rrt_tree(i)),rrt_verts(1,i)],[rrt_verts(2,rrt_tree(i)),rrt_verts(2,i)]);
%         %line([path(1,i),path(1,i + 1)],[path(2,i),path(2,i+1)]);
%     end
    
    
    
    % Check if we have reached goal
    for i = 1:N
        if norm(xy_goal - rrt_tree(i).vertex) < minDistGoal
            if found == 0
                goal_vert = i;
                current_cost = rrt_tree(goal_vert).cost;
                disp('FOUND GOAL')
                disp(N);
                disp('descending cost');
                disp(current_cost);
                best_cost = current_cost;
            else
                if rrt_tree(i).cost < current_cost
                    goal_vert = i;
                    current_cost = rrt_tree(goal_vert).cost;
                    disp('FOUND GOAL')
                    disp(i);
                    disp('descending cost');
                    disp(current_cost);
                    best_cost = [best_cost current_cost];
                end
            end
            found = 1;
            explore = 0;
        end
    end
    
%     if found == 1
%         if rrt_tree(goal_vert).cost < current_cost
%             current_cost = rrt_tree(goal_vert).cost;
%             disp('descending cost');
%             disp(current_cost);
%             disp(N);
%         end
%     end
    
    if N == 2000
        if found
            path = reconstruct_path(rrt_tree,goal_vert);
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
% Plot vertices in RRT
%hold on;
%plot(rrt_verts(1,:),rrt_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);
% figure(2)
% xy_start = [0;0]; plot(xy_start(1),xy_start(2),'bo','MarkerFaceColor','b','MarkerSize',10);
% xy_goal = [pi;0]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10); drawnow;
% plot(final_path(1,:),final_path(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);

figure()
%plot(rrt_verts(1,:),rrt_verts(2,:),'bo','MarkerFaceColor','b','MarkerSize',5);
for i = 1:size(path,2)-1
%for i = 1:N
    %disp(i);
    %if abs(rrt_tree(i).vertex(1) - rrt_tree(rrt_tree(i).parent).vertex(1)) < 0.75*(2*pi)
    if abs(path(1,i) - path(1,i+1)) < 0.75*(2*pi)
        %line([rrt_tree(rrt_tree(i).parent).vertex(1),rrt_tree(i).vertex(1)],[rrt_tree(rrt_tree(i).parent).vertex(2),rrt_tree(i).vertex(2)]);
        %line([closest_vert(1),new_vert(1)],[closest_vert(2),new_vert(2)]);
        line([path(1,i),path(1,i + 1)],[path(2,i),path(2,i+1)]);
    end
    %line([rrt_verts(1,rrt_tree(i)),rrt_verts(1,i)],[rrt_verts(2,rrt_tree(i)),rrt_verts(2,i)]);
    %line([path(1,i),path(1,i + 1)],[path(2,i),path(2,i+1)]);
end

