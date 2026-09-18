function trajectory = createTestTrajectory()
%CREATETESTTRAJECTORY Create a simple synthetic trajectory.

x = linspace(0, 50, 51)';
y = zeros(size(x));

trajectory = [x y];
end