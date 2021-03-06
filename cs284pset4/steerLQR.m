function [new_vert,action,time,cost] = steerLQR(start,goal,K,S,Q,R,T,explore)
%A = [0 1; -9.8*cos(goal(1)) -0.1];
%B = [0;1];
%[K,~] = lqr(A,B,Q,R);
[t,y] = ode45('pendulumLQR',[0 T], start,'mysterious', K, goal);
dimensions = size(y);

%some alternative code
end_point = y(dimensions(1),:).';
if end_point(1) < - pi/2
    end_point(1) = 3*pi/2 + (end_point(1) + pi/2);
end
if end_point(1) > 3*pi/2
    end_point(1) = -pi/2 + end_point(1) - 3*pi/2;
end
new_vert = end_point;
% action = 0;
% time = 0;
% cost = 0;
%end alternative code

minimum = 0;
cost = 0;
if explore
    iter_time = dimensions(1);
else
    iter_time = 0;
    for i = 1:dimensions(1)
        if i == 1
            minimum = norm(y(i,:).' - goal);
            %minimum = (y(i,:).' - goal).'*S*(y(i,:).' - goal);
            end_point = y(i,:).';
            iter_time = i;
        else
            if (y(i,:).' - goal).'*S*(y(i,:).' - goal) < minimum
                minimum = norm(y(i,:).' - goal);
                %minimum = (y(i,:).' - goal).'*S*(y(i,:).' - goal);
                end_point = y(i,:).';
                iter_time = i;
            end
        end
    end
end
%disp(iter_time);
%integrate the dynamics 
action = zeros(1,iter_time-1);
time = t(1:iter_time);
for i = 1:iter_time-1
    x_bar = y(i,:).' - goal;
    u = -K*x_bar;
    if u > 5
        u = 5;
    end
    if u < -5
        u = -5;
    end
    action(i) = u;
    xf = [3.14;0];  %NOT ALWAYS TRUE MAKE SURE YOU ARE AWARE
    cost = cost + ((y(i,:).'-xf).'*Q*(y(i,:).'-xf) + (u.'*R*u))*(t(i+1) - t(i));
    %cost = cost + (u.'*R*u)*(t(i+1) - t(i));
end
%new line of code
end_point = forward_simulate(start,action,time);
%end new line
if end_point(1) < - pi/2
    end_point(1) = 3*pi/2 + (end_point(1) + pi/2);
end
if end_point(1) > 3*pi/2
    end_point(1) = -pi/2 + end_point(1) - 3*pi/2;
end
new_vert = end_point;
return;

