function minimumClearance = M6_CalculateClearance(pathStates,obstacleCenter,obstacleHalfSize)

pathXY = pathStates(:,1:2);

dx = max(abs(pathXY(:,1) - obstacleCenter(1)) - obstacleHalfSize(1),0);
dy = max(abs(pathXY(:,2) - obstacleCenter(2)) - obstacleHalfSize(2),0);

distances = sqrt(dx.^2 + dy.^2);

minimumClearance = min(distances);

end