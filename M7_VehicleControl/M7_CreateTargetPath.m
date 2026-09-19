function [path,heading] = M7_CreateTargetPath()

x = (0:0.25:60)';

y = 5 + ...
    2.5*sin(x/8) + ...
    1.0*sin(x/4);

dx = gradient(x);
dy = gradient(y);

heading = unwrap(atan2(dy,dx));

path = [x y];

end