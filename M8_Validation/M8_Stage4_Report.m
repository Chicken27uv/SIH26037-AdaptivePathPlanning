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

successRate = 100 * mean(resultsTable.Success);
completionRate = 100 * mean(resultsTable.Completed);
collisionFreeRate = 100 * mean(resultsTable.CollisionFree);

averagePredictedClearance = mean(resultsTable.PredictedClearance_m);
averageFinalClearance = mean(resultsTable.FinalClearance_m);
averagePlanningTime = mean(resultsTable.PlanningTime_s);
averageReplanningLatency = mean(resultsTable.ReplanningLatency_s);
averagePathSmoothness = mean(resultsTable.PathSmoothness);
averagePathLength = mean(resultsTable.PathLength_m);
totalReplans = sum(resultsTable.ReplanCount);

summary = table( ...
    successRate, ...
    completionRate, ...
    collisionFreeRate, ...
    averagePredictedClearance, ...
    averageFinalClearance, ...
    averagePlanningTime, ...
    averageReplanningLatency, ...
    averagePathSmoothness, ...
    averagePathLength, ...
    totalReplans);

summary.Properties.VariableNames = { ...
    'M6SuccessRate_pct', ...
    'ScenarioCompletionRate_pct', ...
    'CollisionFreeRate_pct', ...
    'AveragePredictedClearance_m', ...
    'AverageFinalClearance_m', ...
    'AveragePlanningTime_s', ...
    'AverageReplanningLatency_s', ...
    'AveragePathSmoothness', ...
    'AveragePathLength_m', ...
    'TotalReplans'};

if ~isfolder("Results")
    mkdir("Results")
end

if ~isfolder("Figures")
    mkdir("Figures")
end

writetable( ...
    resultsTable, ...
    fullfile("Results","M8_Stage4_ScenarioResults.csv"))

writetable( ...
    summary, ...
    fullfile("Results","M8_Stage4_Summary.csv"))

save( ...
    fullfile("Results","M8_Stage4_ValidationResults.mat"), ...
    "resultsTable", ...
    "summary")

figure

tiledlayout(2,2)

nexttile

bar( ...
    categorical(resultsTable.Scenario), ...
    [ ...
    resultsTable.PredictedClearance_m ...
    resultsTable.FinalClearance_m])

title("Clearance")
xlabel("Scenario")
ylabel("Meters")
legend("Predicted","Final","Location","best")
grid on

nexttile

bar( ...
    categorical(resultsTable.Scenario), ...
    resultsTable.ReplanningLatency_s)

title("Replanning Latency")
xlabel("Scenario")
ylabel("Seconds")
grid on

nexttile

bar( ...
    categorical(resultsTable.Scenario), ...
    resultsTable.PathLength_m)

title("Selected Path Length")
xlabel("Scenario")
ylabel("Meters")
grid on

nexttile

bar( ...
    categorical(resultsTable.Scenario), ...
    resultsTable.PathSmoothness)

title("Path Smoothness")
xlabel("Scenario")
ylabel("Smoothness")
grid on

sgtitle("M8 Stage 4 Closed-Loop Validation Dashboard")

exportgraphics( ...
    gcf, ...
    fullfile("Figures","M8_Stage4_ValidationDashboard.png"), ...
    "Resolution",200)

fprintf("\nM8 STAGE 4 VALIDATION REPORT\n")
fprintf("-----------------------------\n")

disp(resultsTable)

fprintf("\nOVERALL VALIDATION SUMMARY\n")
fprintf("---------------------------\n")
fprintf("M6 success rate:                 %.1f %%\n",successRate)
fprintf("Scenario completion rate:        %.1f %%\n",completionRate)
fprintf("Collision-free rate:             %.1f %%\n",collisionFreeRate)
fprintf("Average predicted clearance:     %.2f m\n",averagePredictedClearance)
fprintf("Average final clearance:         %.2f m\n",averageFinalClearance)
fprintf("Average planning time:           %.4f s\n",averagePlanningTime)
fprintf("Average replanning latency:      %.4f s\n",averageReplanningLatency)
fprintf("Average path smoothness:         %.4f\n",averagePathSmoothness)
fprintf("Average path length:              %.2f m\n",averagePathLength)
fprintf("Total replans:                   %d\n",totalReplans)

fprintf("\nSaved files:\n")
fprintf("Results/M8_Stage4_ScenarioResults.csv\n")
fprintf("Results/M8_Stage4_Summary.csv\n")
fprintf("Results/M8_Stage4_ValidationResults.mat\n")
fprintf("Figures/M8_Stage4_ValidationDashboard.png\n")