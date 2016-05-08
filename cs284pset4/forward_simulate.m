function [approx_vert,intermediate] = forward_simulate(start,action,time)
    approx_vert = start;
    intermediate = zeros(2,size(action,1)+1);
    intermediate(:,1) = start;
    for i = 1:size(action,2)
        theta_d = approx_vert(2);
        theta_dd = action(i) -9.8*sin(approx_vert(1)) - 0.1*approx_vert(2);
        approx_vert(1) = approx_vert(1) + theta_d*(time(i+1)-time(i));
        approx_vert(2) = approx_vert(2) + theta_dd*(time(i+1)-time(i));
        if approx_vert(1) < - pi/2
            approx_vert(1) = 3*pi/2 + (approx_vert(1) + pi/2);
        end
        if approx_vert(1) > 3*pi/2
            approx_vert(1) = -pi/2 + approx_vert(1) - 3*pi/2;
        end
        intermediate(:,i+1) = approx_vert;
    end
end

