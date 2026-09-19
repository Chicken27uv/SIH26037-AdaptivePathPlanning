function risk = M5_CalculateRisk(ego,object)

positionDifference = object.position - ego.position;
distance = norm(positionDifference);

relativeVelocity = object.velocity - ego.velocity;

if distance > eps
    closingSpeed = max(0,-dot(relativeVelocity,positionDifference/distance));
else
    closingSpeed = norm(relativeVelocity);
end

if isfield(object,"predictedTrajectory") && ~isempty(object.predictedTrajectory)

    predictedTrajectory = object.predictedTrajectory;

    predictionCount = size(predictedTrajectory,1);

    if predictionCount > 1

        dt = 0.5;
        predictionTime = (0:predictionCount-1)' * dt;

        egoPredictedTrajectory = ...
            ego.position + predictionTime .* ego.velocity;

        synchronizedDistances = vecnorm( ...
            predictedTrajectory - egoPredictedTrajectory,2,2);

        minimumPredictedDistance = min(synchronizedDistances);

        conflictIndex = ...
            find(synchronizedDistances == minimumPredictedDistance,1);

    else

        minimumPredictedDistance = ...
            norm(predictedTrajectory(1,:) - ego.position);

        conflictIndex = 1;

    end

else

    minimumPredictedDistance = distance;
    conflictIndex = 1;

end

if isfield(object,"safetyMargin")

    safetyMargin = object.safetyMargin;

else

    switch string(object.class)

        case {"pedestrian","cattle","bicycle","pushcart"}
            safetyMargin = 3.0;

        case {"two-wheeler","auto-rickshaw"}
            safetyMargin = 2.5;

        otherwise
            safetyMargin = 2.0;

    end

end

trajectoryConflict = minimumPredictedDistance <= safetyMargin;

nearConflict = minimumPredictedDistance <= 2*safetyMargin;

if trajectoryConflict

    riskLevel = 3;

elseif nearConflict && closingSpeed > 1.0

    riskLevel = 2;

elseif nearConflict

    riskLevel = 2;

else

    riskLevel = 1;

end

risk.class = string(object.class);
risk.distance = distance;
risk.closingSpeed = closingSpeed;
risk.minimumPredictedDistance = minimumPredictedDistance;
risk.safetyMargin = safetyMargin;
risk.trajectoryConflict = trajectoryConflict;
risk.conflictIndex = conflictIndex;
risk.level = riskLevel;

end