function [new_vert,cost,iter_time] = steerLQR(start,goal,K,S,Q,R)
new_vert = [0;0];
full_time = 0.2; %just a constant
%A = [0 1; -9.8*cos(goal(1)) -0.1];
%B = [0;1];
%[K,~] = lqr(A,B,Q,R);
[t,y] = ode45('pendulumLQR',[0 0.2], start,'mysterious', K.', goal);
dimensions = size(y);
minimum = 0;
cost = 0;
iter_time = 0.0;
for i = 1:dimensions(1)
    if i == 1
        %minimum = norm(y(i,:).' - goal);
        minimum = (y(i,:).' - goal).'*S*(y(i,:).' - goal);
        end_point = y(i,:);
        iter_time = i;
    else
        if (y(i,:).' - goal).'*S*(y(i,:).' - goal) < minimum
            minimum = (y(i,:).' - goal).'*S*(y(i,:).' - goal);
            end_point = y(i,:);
            iter_time = i;
        end
    end
end

%integrate the dynamics 
time_step = t(2,:) - t(1,:);
for i = 1:iter_time
    x_bar = y(i,:).' - goal;
    u = -K.'*x_bar;
    cost = cost + (x_bar.'*Q*x_bar + u.'*R*u)*time_step;
end

if end_point(1) < - pi/2
    end_point(1) = 3*pi/2 + (end_point(1) + pi/2);
end
if end_point(1) > 3*pi/2
    end_point(1) = -pi/2 + end_point(1) - 3*pi/2;
end
new_vert = end_point;
return;

