%% SIH 26037
% Module M4 - Short-Term Motion Prediction
% Developer: Daniel
%
% Stage M4-E:
% Final class-aware hybrid motion predictor
%
% Strategy:
% - Stable / highly irregular actors -> Constant Velocity
% - Informal merging/drifting vehicle -> Constant Acceleration
%
% This is a lightweight prototype heuristic.

clc;
clear;
close all;

%% Settings
historyStep = 0.2;
historyDuration = 1.0;

predictionStep = 0.2;
predictionHorizon = 2.0;

historyTimes = -historyDuration:historyStep:0;
futureTimes = 0:predictionStep:predictionHorizon;

%% Initial object states
objects(1).id = 1;
objects(1).type = "Car";
objects(1).state = [20, 2.5, -1.0, 0.0];

objects(2).id = 2;
objects(2).type = "Pedestrian";
objects(2).state = [12, 7.0, 0.0, -0.8];

objects(3).id = 3;
objects(3).type = "Auto-rickshaw";
objects(3).state = [8, -4.0, 2.0, 0.25];

objects(4).id = 4;
objects(4).type = "Two-wheeler";
objects(4).state = [-5, -2.0, 5.0, 0.15];

objects(5).id = 5;
objects(5).type = "Cattle";
objects(5).state = [25, -7.0, -0.15, 0.7];

numObjects = length(objects);

%% Metrics
baselineRMSE = zeros(1,numObjects);
finalRMSE = zeros(1,numObjects);

baselineFinalError = zeros(1,numObjects);
finalFinalError = zeros(1,numObjects);

selectedModel = strings(1,numObjects);

%% Visualization
figure( ...
    'Name','SIH 26037 - M4-E Final Prediction', ...
    'NumberTitle','off');

hold on;
grid on;

patch([-10 55 55 -10], ...
      [-8 -8 8 8], ...
      [0.85 0.85 0.85], ...
      'EdgeColor','none');

%% Process every actor
for i = 1:numObjects

    x0 = objects(i).state(1);
    y0 = objects(i).state(2);
    vx0 = objects(i).state(3);
    vy0 = objects(i).state(4);

    %% -------------------------------------------------
    % Generate recent motion history
    %% -------------------------------------------------

    historyX = zeros(size(historyTimes));
    historyY = zeros(size(historyTimes));

    historyX(1) = x0 - vx0*historyDuration;
    historyY(1) = y0 - vy0*historyDuration;

    for k = 2:length(historyTimes)

        t = historyTimes(k);
        dt = historyStep;

        switch objects(i).type

            case "Car"
                vx = -1.0;
                vy = 0.0;

            case "Pedestrian"
                vx = 0.15*sin(1.5*t);
                vy = -0.8 - 0.15*sin(2*t);

            case "Auto-rickshaw"
                vx = 2.0;
                vy = 0.25 + 0.35*sin(1.3*t);

            case "Two-wheeler"
                vx = 5.0;
                vy = 0.15 + 0.25*sin(2.5*t);

            case "Cattle"
                vx = -0.15 + 0.15*sin(t);
                vy = 0.7 + 0.30*sin(1.8*t);
        end

        historyX(k) = historyX(k-1) + vx*dt;
        historyY(k) = historyY(k-1) + vy*dt;
    end

    %% Current estimated state
    currentX = historyX(end);
    currentY = historyY(end);

    vxRecent = ...
        (historyX(end) - historyX(end-1)) / historyStep;

    vyRecent = ...
        (historyY(end) - historyY(end-1)) / historyStep;

    vxPrevious = ...
        (historyX(end-1) - historyX(end-2)) / historyStep;

    vyPrevious = ...
        (historyY(end-1) - historyY(end-2)) / historyStep;

    %% Acceleration estimate
    axEstimate = ...
        (vxRecent - vxPrevious) / historyStep;

    ayEstimate = ...
        (vyRecent - vyPrevious) / historyStep;

    %% -------------------------------------------------
    % BASELINE CONSTANT-VELOCITY MODEL
    %% -------------------------------------------------

    baselineX = ...
        currentX + vxRecent .* futureTimes;

    baselineY = ...
        currentY + vyRecent .* futureTimes;

    %% -------------------------------------------------
    % CONSTANT-ACCELERATION CANDIDATE
    %% -------------------------------------------------

    accelerationX = ...
        currentX + ...
        vxRecent .* futureTimes + ...
        0.5 .* axEstimate .* futureTimes.^2;

    accelerationY = ...
        currentY + ...
        vyRecent .* futureTimes + ...
        0.5 .* ayEstimate .* futureTimes.^2;

    %% -------------------------------------------------
    % CLASS-AWARE MODEL SELECTION
    %% -------------------------------------------------

    switch objects(i).type

        case "Auto-rickshaw"

            % Informal merge/drift:
            % acceleration model captures changing lateral motion
            finalX = accelerationX;
            finalY = accelerationY;

            selectedModel(i) = ...
                "Acceleration / Drift";

        otherwise

            % Conservative forecast for:
            % car, pedestrian, two-wheeler, cattle
            finalX = baselineX;
            finalY = baselineY;

            selectedModel(i) = ...
                "Constant Velocity";
    end

    %% -------------------------------------------------
    % Generate future ground truth
    %% -------------------------------------------------

    trueX = zeros(size(futureTimes));
    trueY = zeros(size(futureTimes));

    trueX(1) = currentX;
    trueY(1) = currentY;

    for k = 2:length(futureTimes)

        t = futureTimes(k);
        dt = predictionStep;

        switch objects(i).type

            case "Car"
                vx = -1.0;
                vy = 0.0;

            case "Pedestrian"
                vx = 0.15*sin(1.5*t);
                vy = -0.8 - 0.15*sin(2*t);

            case "Auto-rickshaw"
                vx = 2.0;
                vy = 0.25 + 0.35*sin(1.3*t);

            case "Two-wheeler"
                vx = 5.0;
                vy = 0.15 + 0.25*sin(2.5*t);

            case "Cattle"
                vx = -0.15 + 0.15*sin(t);
                vy = 0.7 + 0.30*sin(1.8*t);
        end

        trueX(k) = ...
            trueX(k-1) + vx*dt;

        trueY(k) = ...
            trueY(k-1) + vy*dt;
    end

    %% -------------------------------------------------
    % Errors
    %% -------------------------------------------------

    baselineErrors = hypot( ...
        baselineX - trueX, ...
        baselineY - trueY);

    finalErrors = hypot( ...
        finalX - trueX, ...
        finalY - trueY);

    baselineRMSE(i) = ...
        sqrt(mean(baselineErrors.^2));

    finalRMSE(i) = ...
        sqrt(mean(finalErrors.^2));

    baselineFinalError(i) = ...
        baselineErrors(end);

    finalFinalError(i) = ...
        finalErrors(end);

    %% -------------------------------------------------
    % Visualization
    %% -------------------------------------------------

    plot(historyX,historyY, ...
        'k-', ...
        'LineWidth',1.2);

    plot(currentX,currentY, ...
        'ko', ...
        'MarkerFaceColor','k');

    plot(baselineX,baselineY, ...
        '--', ...
        'LineWidth',1.2);

    plot(finalX,finalY, ...
        '-.', ...
        'LineWidth',1.8);

    plot(trueX,trueY, ...
        '-', ...
        'LineWidth',2);

    text( ...
        currentX+0.4, ...
        currentY+0.4, ...
        sprintf( ...
        'T%d | %s | %s', ...
        objects(i).id, ...
        objects(i).type, ...
        selectedModel(i)), ...
        'FontWeight','bold');
end

%% Formatting
xlim([-10 55]);
ylim([-12 12]);

axis equal;

xlabel('Longitudinal Position X (m)');
ylabel('Lateral Position Y (m)');

title('M4-E: Final Class-Aware Short-Term Prediction');

%% -------------------------------------------------
% Results
%% -------------------------------------------------

fprintf('\n');
fprintf('==========================================================================\n');
fprintf(' SIH 26037 - M4-E FINAL MOTION PREDICTION RESULTS\n');
fprintf('==========================================================================\n');

fprintf('%-16s %-22s %-12s %-12s %-12s %-12s\n', ...
    'Object', ...
    'Selected Model', ...
    'Base RMSE', ...
    'Final RMSE', ...
    'Base Final', ...
    'Final Error');

fprintf('--------------------------------------------------------------------------\n');

for i = 1:numObjects

    fprintf('%-16s %-22s %8.3f m   %8.3f m   %8.3f m   %8.3f m\n', ...
        objects(i).type, ...
        selectedModel(i), ...
        baselineRMSE(i), ...
        finalRMSE(i), ...
        baselineFinalError(i), ...
        finalFinalError(i));
end

fprintf('==========================================================================\n');

overallBaselineRMSE = ...
    sqrt(mean(baselineRMSE.^2));

overallFinalRMSE = ...
    sqrt(mean(finalRMSE.^2));

improvementPercent = ...
    100 * ...
    (overallBaselineRMSE - overallFinalRMSE) / ...
    overallBaselineRMSE;

fprintf('\nOverall baseline RMSE : %.3f m\n', ...
    overallBaselineRMSE);

fprintf('Overall final RMSE    : %.3f m\n', ...
    overallFinalRMSE);

fprintf('Overall improvement   : %.2f %%\n', ...
    improvementPercent);

%% -------------------------------------------------
% Comparison chart
%% -------------------------------------------------

figure( ...
    'Name','M4-E Final Prediction Comparison', ...
    'NumberTitle','off');

comparisonData = [
    baselineRMSE(:), ...
    finalRMSE(:)
];

bar(comparisonData);

set(gca, ...
    'XTick',1:numObjects, ...
    'XTickLabel',{objects.type});

ylabel('Prediction RMSE (m)');
xlabel('Road User');

legend( ...
    {'Constant Velocity Baseline', ...
     'Final Class-Aware Predictor'}, ...
    'Location','best');

title('M4-E Prediction Performance Comparison');

grid on;

disp("M4-E final motion prediction completed successfully.");