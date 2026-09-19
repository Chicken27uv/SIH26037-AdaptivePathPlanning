function result = M8_RunScenario(scenario)

t = scenario.time;

egoX = scenario.startPose(1) + scenario.egoSpeed .* t;

s = zeros(size(t));

active = t >= scenario.avoidanceStart & t <= scenario.avoidanceEnd;

s(active) = ...
    (t(active) - scenario.avoidanceStart) ./ ...
    (scenario.avoidanceEnd - scenario.avoidanceStart);

s = s .* (s <= 1);

offset = scenario.avoidanceOffset .* ...
    (3*s.^2 - 2*s.^3);

returnActive = t > scenario.avoidanceEnd;

if any(returnActive)
    sr = ...
        (t(returnActive) - scenario.avoidanceEnd) ./ ...
        (scenario.duration - scenario.avoidanceEnd);

    offset(returnActive) = scenario.avoidanceOffset .* ...
        (1 - (3*sr.^2 - 2*sr.^3));
end

egoY = scenario.startPose(2) + offset;

egoTrajectory = [egoX' egoY'];

objectX = scenario.obstacleStart(1) + ...
    scenario.obstacleVelocity(1) .* t;

objectY = scenario.obstacleStart(2) + ...
    scenario.obstacleVelocity(2) .* t;

objectTrajectory = [objectX' objectY'];

distances = vecnorm(egoTrajectory - objectTrajectory,2,2);

[minClearance,minIndex] = min(distances);

collision = minClearance < scenario.minimumSafeClearance;

goalDistance = norm( ...
    egoTrajectory(end,:) - scenario.goalPoint);

completion = goalDistance < 1.0;

result.name = scenario.name;
result.time = t;
result.egoTrajectory = egoTrajectory;
result.objectTrajectory = objectTrajectory;
result.minimumClearance = minClearance;
result.minimumClearanceTime = t(minIndex);
result.collision = collision;
result.collisionFree = ~collision;
result.goalDistance = goalDistance;
result.completed = completion;
result.replanningLatency = scenario.avoidanceStart;
result.pathSmoothness = 0;
result.replanCount = 1;

end