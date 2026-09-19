clear
clc
close all

scenarioNames = [
    "VillageRoad"
    "UrbanIntersection"
    "HighwayMerge"
    "MarketArea"
    "CattleCrossing"
];

results = struct([]);

for i = 1:numel(scenarioNames)

    input = M8_CreateM6ValidationInput(scenarioNames(i));

    output = M6_AdaptivePathPlanner(input);

    metrics = M8_CalculateM6Metrics(output,input);

    results(i).Scenario = scenarioNames(i);
    results(i).ObjectClass = input.predictedObjects(1).class;
    results(i).Success = metrics.success;
    results(i).Completed = metrics.completed;
    results(i).CollisionFree = metrics.collisionFree;
    results(i).PredictedClearance_m = metrics.predictedClearance;
    results(i).FinalClearance_m = metrics.minimumClearance;
    results(i).PlanningTime_s = metrics.planningTime;
    results(i).ReplanningLatency_s = metrics.replanningLatency;
    results(i).PathLength_m = metrics.pathLength;
    results(i).PathSmoothness = metrics.pathSmoothness;
    results(i).ReplanCount = metrics.replanCount;
    results(i).GoalDistance_m = metrics.goalDistance;
    results(i).ReplanRequired = metrics.replanRequired;

end

resultsTable = struct2table(results);

completionRate = ...
    100 * mean(resultsTable.Completed);

collisionFreeRate = ...
    100 * mean(resultsTable.CollisionFree);

successRate = ...
    100 * mean(resultsTable.Success);

averagePredictedClearance = ...
    mean(resultsTable.PredictedClearance_m);

averageFinalClearance = ...
    mean(resultsTable.FinalClearance_m);

averagePlanningTime = ...
    mean(resultsTable.PlanningTime_s);

averageReplanningLatency = ...
    mean(resultsTable.ReplanningLatency_s);

totalReplans = ...
    sum(resultsTable.ReplanCount);

fprintf("\nM8 STAGE 3 M6 VALIDATION\n")
fprintf("-------------------------\n")

disp(resultsTable)

fprintf("M6 success rate:                 %.1f %%\n",successRate)
fprintf("Scenario completion rate:        %.1f %%\n",completionRate)
fprintf("Collision-free rate:             %.1f %%\n",collisionFreeRate)
fprintf("Average predicted clearance:     %.2f m\n",averagePredictedClearance)
fprintf("Average final clearance:         %.2f m\n",averageFinalClearance)
fprintf("Average planning time:           %.4f s\n",averagePlanningTime)
fprintf("Average replanning latency:      %.4f s\n",averageReplanningLatency)
fprintf("Total replans:                   %d\n",totalReplans)

figure

bar(categorical(resultsTable.Scenario), ...
    [resultsTable.PredictedClearance_m resultsTable.FinalClearance_m])

title("M8 M6 Validation Clearance")
xlabel("Scenario")
ylabel("Clearance [meters]")
legend("Predicted Clearance","Final Clearance","Location","best")
grid on

figure

bar(categorical(resultsTable.Scenario), ...
    resultsTable.ReplanningLatency_s)

title("M8 M6 Replanning Latency")
xlabel("Scenario")
ylabel("Latency [seconds]")
grid on

figure

bar(categorical(resultsTable.Scenario), ...
    resultsTable.PathLength_m)

title("M8 M6 Selected Path Length")
xlabel("Scenario")
ylabel("Path Length [meters]")
grid on

figure

bar(categorical(resultsTable.Scenario), ...
    resultsTable.PathSmoothness)

title("M8 M6 Path Smoothness")
xlabel("Scenario")
ylabel("Smoothness")
grid on