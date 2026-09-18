function [predictedTrajectory,minPredictedClearance,predictedConflictPoint,replanRequired,triggerIndex,triggerPoint] = M6_PredictConflict(pathStates,object,predictionHorizon,replanThreshold)

predictionDt = 0.1;
predictionTime = 0:predictionDt:predictionHorizon;

predictedTrajectory = object.position + ...
    predictionTime' * object.velocity;

distances = zeros(size(predictedTrajectory,1),1);
pathIndices = zeros(size(predictedTrajectory,1),1);

for k = 1:size(predictedTrajectory,1)
    [distances(k),pathIndices(k)] = min(vecnorm( ...
        pathStates(:,1:2) - predictedTrajectory(k,:),2,2));
end

[minPredictedClearance,predictionIndex] = min(distances);
predictedConflictPoint = predictedTrajectory(predictionIndex,:);

triggerIndex = pathIndices(predictionIndex);
triggerPoint = pathStates(triggerIndex,:);

replanRequired = minPredictedClearance < replanThreshold;

end