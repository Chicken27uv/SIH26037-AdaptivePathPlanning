%% SIH 26037
% Module M4 - Short-Term Motion Prediction
% Developer: Daniel
%
% Stage M4-B:
% Prediction error evaluation

clc;
clear;
close all;

%% Prediction settings
predictionHorizon = 2.0;
predictionStep = 0.2;

futureTimes = 0:predictionStep:predictionHorizon;

%% Initial object states
% [x, y, vx, vy]

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

%% Metric storage
meanErrors = zeros(1,numObjects);
rmseErrors = zeros(1,numObjects);
finalErrors = zeros(1,numObjects);

%% Main visualization
figure( ...
    'Name','SIH 26037 - M4-B Prediction Evaluation', ...
    'NumberTitle','off');

hold on;
grid on;

patch([-10 55 55 -10], ...
      [-8 -8 8 8], ...
      [0.85 0.85 0.85], ...
      'EdgeColor','none');

%% Process each object
for i = 1:numObjects

    x = objects(i).state(1);
    y = objects(i).state(2);
    vx = objects(i).state(3);
    vy = objects(i).state(4);

    %% --------------------------------
    % CONSTANT-VELOCITY PREDICTION
    %% --------------------------------

    predictedX = x + vx .* futureTimes;
    predictedY = y + vy .* futureTimes;

    %% --------------------------------
    % SIMULATED FUTURE GROUND TRUTH
    %% --------------------------------

    trueX = zeros(size(futureTimes));
    trueY = zeros(size(futureTimes));

    trueX(1) = x;
    trueY(1) = y;

    for k = 2:length(futureTimes)

        dt = predictionStep;

        currentTime = futureTimes(k);

        switch objects(i).type

            case "Car"
                % Approximately constant movement
                trueVx = vx;
                trueVy = vy;

            case "Pedestrian"
                % Slight lateral irregularity
                trueVx = 0.15*sin(1.5*currentTime);
                trueVy = -0.8 - 0.15*sin(2*currentTime);

            case "Auto-rickshaw"
                % Informal lateral merge/drift
                trueVx = 2.0;
                trueVy = 0.25 + 0.35*sin(1.3*currentTime);

            case "Two-wheeler"
                % Minor lateral weaving
                trueVx = 5.0;
                trueVy = 0.15 + 0.25*sin(2.5*currentTime);

            case "Cattle"
                % Irregular crossing
                trueVx = -0.15 + 0.15*sin(currentTime);
                trueVy = 0.7 + 0.30*sin(1.8*currentTime);
        end

        trueX(k) = trueX(k-1) + trueVx*dt;
        trueY(k) = trueY(k-1) + trueVy*dt;
    end

    %% --------------------------------
    % ERROR CALCULATION
    %% --------------------------------

    errors = hypot( ...
        predictedX - trueX, ...
        predictedY - trueY);

    meanErrors(i) = mean(errors);

    rmseErrors(i) = ...
        sqrt(mean(errors.^2));

    finalErrors(i) = errors(end);

    %% --------------------------------
    % VISUALIZATION
    %% --------------------------------

    plot(x,y,'ko', ...
        'MarkerSize',9, ...
        'MarkerFaceColor','k');

    plot(predictedX,predictedY,'--o', ...
        'LineWidth',1.5, ...
        'MarkerSize',4);

    plot(trueX,trueY,'-', ...
        'LineWidth',2);

    text( ...
        x+0.4, ...
        y+0.4, ...
        sprintf('T%d | %s', ...
        objects(i).id, ...
        objects(i).type), ...
        'FontWeight','bold');
end

%% Figure formatting
xlim([-10 55]);
ylim([-12 12]);

axis equal;

xlabel('Longitudinal Position X (m)');
ylabel('Lateral Position Y (m)');

title( ...
    'M4-B: Predicted Trajectory vs Future Ground Truth');

%% Print results
fprintf('\n');
fprintf('============================================================\n');
fprintf(' SIH 26037 - M4-B PREDICTION ERROR RESULTS\n');
fprintf('============================================================\n');

fprintf('%-16s %-12s %-12s %-12s\n', ...
    'Object', ...
    'Mean Error', ...
    'RMSE', ...
    'Final Error');

fprintf('------------------------------------------------------------\n');

for i = 1:numObjects

    fprintf('%-16s %8.3f m   %8.3f m   %8.3f m\n', ...
        objects(i).type, ...
        meanErrors(i), ...
        rmseErrors(i), ...
        finalErrors(i));
end

fprintf('============================================================\n');

%% Overall metrics
overallMeanError = mean(meanErrors);

overallRMSE = sqrt(mean(rmseErrors.^2));

fprintf('\nOverall mean prediction error : %.3f m\n', ...
    overallMeanError);

fprintf('Overall prediction RMSE       : %.3f m\n', ...
    overallRMSE);

%% Error summary figure
figure( ...
    'Name','M4-B Prediction Error Summary', ...
    'NumberTitle','off');

bar(rmseErrors);

set(gca, ...
    'XTick',1:numObjects, ...
    'XTickLabel',{objects.type});

ylabel('Prediction RMSE (m)');
xlabel('Road User');

title('M4-B Prediction RMSE by Road User');

grid on;

disp("M4-B prediction evaluation completed successfully.");