function output = M5_DecisionLogic(input)

ego = input.ego;
objects = input.objects;

startTime = tic;

if isempty(objects)

    output.behavior = "CRUISE";
    output.replanRequired = false;
    output.targetSpeed = min(norm(ego.velocity),5.0);
    output.riskLevel = "LOW";
    output.targetObject = "";
    output.reason = "No surrounding objects";

    output.metrics.numberOfObjects = 0;
    output.metrics.numberOfConflicts = 0;
    output.metrics.highestRiskLevel = 1;
    output.metrics.decisionLatency = toc(startTime);

    return

end

riskTemplate = M5_CalculateRisk(ego,objects(1));
risks = repmat(riskTemplate,1,numel(objects));

for i = 1:numel(objects)
    risks(i) = M5_CalculateRisk(ego,objects(i));
end

levels = [risks.level];
minimumPredictedDistances = [risks.minimumPredictedDistance];

selectionMatrix = [ ...
    levels(:), ...
    -minimumPredictedDistances(:)];

[~,sortedIndices] = sortrows(selectionMatrix,[-1 -2]);

targetIndex = sortedIndices(1);
highestLevel = risks(targetIndex).level;

if highestLevel == 1

    output.behavior = "CRUISE";
    output.replanRequired = false;
    output.targetSpeed = min(norm(ego.velocity),5.0);
    output.riskLevel = "LOW";
    output.targetObject = "";
    output.reason = "No immediate collision conflict";

else

    selectedDecision = M5_SelectBehavior( ...
        risks(targetIndex), ...
        ego, ...
        objects(targetIndex));

    switch highestLevel

        case 2

            if risks(targetIndex).minimumPredictedDistance <= ...
                    0.75*risks(targetIndex).safetyMargin

                riskLevel = "HIGH";

            else

                riskLevel = "MEDIUM";

            end

        otherwise

            if risks(targetIndex).minimumPredictedDistance <= ...
                    0.75*risks(targetIndex).safetyMargin

                riskLevel = "CRITICAL";

            else

                riskLevel = "HIGH";

            end

    end

    if selectedDecision.replanRequired

        reason = "Predicted conflict requires adaptive response";

    elseif selectedDecision.behavior == "FOLLOW"

        reason = "Object ahead requires reduced speed";

    else

        reason = "No immediate collision conflict";

    end

    output.behavior = selectedDecision.behavior;
    output.replanRequired = selectedDecision.replanRequired;
    output.targetSpeed = selectedDecision.targetSpeed;
    output.riskLevel = riskLevel;
    output.targetObject = risks(targetIndex).class;
    output.reason = reason;

end

output.metrics.numberOfObjects = numel(objects);
output.metrics.numberOfConflicts = sum(levels >= 2);
output.metrics.highestRiskLevel = highestLevel;
output.metrics.decisionLatency = toc(startTime);

if highestLevel >= 2

    output.selectedRisk = risks(targetIndex);

else

    output.selectedRisk = risks(1);

end

output.allRisks = risks;

end