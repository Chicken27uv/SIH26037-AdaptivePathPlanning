function metrics = M6_CalculateMetrics(initialStates,replannedStates,initialPlanningTime,replanningLatency,minPredictedClearance,replanCount,map,obstacleCenter,obstacleHalfSize)

initialXY = initialStates(:,1:2);
replannedXY = replannedStates(:,1:2);

initialPathLength = sum(vecnorm(diff(initialXY),2,2));
replannedPathLength = sum(vecnorm(diff(replannedXY),2,2));

initialHeading = unwrap(initialStates(:,3));
replannedHeading = unwrap(replannedStates(:,3));

initialHeadingChange = diff(initialHeading);
replannedHeadingChange = diff(replannedHeading);

initialSmoothness = sum(initialHeadingChange.^2);
replannedSmoothness = sum(replannedHeadingChange.^2);

initialOccupancy = getOccupancy(map,initialXY);
replannedOccupancy = getOccupancy(map,replannedXY);

initialMapCollision = any(initialOccupancy > 0.5);
replannedMapCollision = any(replannedOccupancy > 0.5);

dx = max(abs(replannedXY(:,1) - obstacleCenter(1)) - obstacleHalfSize(1),0);
dy = max(abs(replannedXY(:,2) - obstacleCenter(2)) - obstacleHalfSize(2),0);

replannedObstacleDistance = sqrt(dx.^2 + dy.^2);
replannedObstacleClearance = min(replannedObstacleDistance);

metrics.initialPlanningTime = initialPlanningTime;
metrics.replanningLatency = replanningLatency;
metrics.initialPathLength = initialPathLength;
metrics.replannedPathLength = replannedPathLength;
metrics.initialSmoothness = initialSmoothness;
metrics.replannedSmoothness = replannedSmoothness;
metrics.minimumPredictedClearance = minPredictedClearance;
metrics.replannedObstacleClearance = replannedObstacleClearance;
metrics.replanCount = replanCount;
metrics.initialMapCollision = initialMapCollision;
metrics.replannedMapCollision = replannedMapCollision;
metrics.collisionFree = ~replannedMapCollision;

end