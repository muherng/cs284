function [dy] = pendulumcats(t,y,weird,u)
dy = zeros(2,1);
dy(1) = y(2);
dy(2) = u -9.8*sin(y(1)) - 0.1*y(2);
end

