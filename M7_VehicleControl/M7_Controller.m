function [steeringCommand,speedCommand,targetIndex,crossTrackError] = M7_Controller( ...
    vehicleState,targetPath,targetHeading,previousIndex)

wheelbase = 2.7;
lookaheadDistance = 4.0;
maxSteering = deg2rad(35);
targetSpeed = 5.0;

position = vehicleState(1:2);
heading = vehicleState(3);

distances = vecnorm(targetPath - position,2,2);

searchStart = max(1,min(previousIndex,size(targetPath,1)));
[~,localIndex] = min(distances(searchStart:end));
nearestIndex = searchStart + localIndex - 1;

lookaheadIndex = nearestIndex;

while lookaheadIndex < size(targetPath,1)

    distanceToTarget = norm( ...
        targetPath(lookaheadIndex,:) - position);

    if distanceToTarget >= lookaheadDistance
        break
    end

    lookaheadIndex = lookaheadIndex + 1;

end

goalPoint = targetPath(end,:);
goalDistance = norm(goalPoint - position);

if goalDistance < 0.8

    steeringCommand = 0;
    speedCommand = 0;
    targetIndex = size(targetPath,1);
    crossTrackError = 0;
    return

end

lookaheadPoint = targetPath(lookaheadIndex,:);

alpha = atan2( ...
    lookaheadPoint(2) - position(2), ...
    lookaheadPoint(1) - position(1)) - heading;

alpha = atan2(sin(alpha),cos(alpha));

steeringCommand = atan2( ...
    2 * wheelbase * sin(alpha), ...
    lookaheadDistance);

steeringCommand = max( ...
    -maxSteering, ...
    min(maxSteering,steeringCommand));

pathHeading = targetHeading(nearestIndex);

headingError = atan2( ...
    sin(pathHeading - heading), ...
    cos(pathHeading - heading));

crossTrackVector = position - targetPath(nearestIndex,:);

crossTrackError = ...
    -sin(pathHeading) * crossTrackVector(1) + ...
     cos(pathHeading) * crossTrackVector(2);

speedCommand = min(targetSpeed,2.0 * goalDistance);

if abs(headingError) > deg2rad(20)
    speedCommand = min(speedCommand,3.0);
end

if goalDistance < 2.0
    speedCommand = min(speedCommand,1.5);
end

targetIndex = nearestIndex;

end