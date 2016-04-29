disp('Hello');
x = randn(2,3);
y = randn(2,3);
ch = convhull(y(1,:),y(2,:));
polygon = zeros(2,3);
for i = 1:3
    polygon(:,i) = y(:,ch(i));
end
in = inpolygon(x(1,:),x(2,:),polygon(1,:),polygon(2,:));
disp(in);