%% SIH 26037
% Module M4 - Short-Term Motion Prediction
% Developer: Daniel
%
% Stage M4-A:
% Constant-velocity trajectory prediction baseline

clc;
clear;
close all;

%% Prediction settings
dt = 0.1;                  % simulation step (s)
predictionHorizon = 2.0;   % seconds
predictionStep = 0.2;      % future prediction interval (s)

futureTimes = 0:predictionStep:predictionHorizon;

%% Current tracked-object states
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

%% Store predictions
predictions = cell(1,numObjects);

%% Create figure
figure( ...
    'Name','SIH 26037 - M4-A Motion Prediction', ...
    'NumberTitle','off');

hold on;
grid on;

%% Draw road
patch([-10 55 55 -10], ...
      [-8 -8 8 8], ...
      [0.85 0.85 0.85], ...
      'EdgeColor','none');

%% Predict trajectory for every object
for i = 1:numObjects

    x = objects(i).state(1);
    y = objects(i).state(2);

    vx = objects(i).state(3);
    vy = objects(i).state(4);

    predictedX = x + vx .* futureTimes;
    predictedY = y + vy .* futureTimes;

    predictions{i} = [predictedX(:), predictedY(:)];

    %% Plot current position
    plot(x,y,'ko', ...
        'MarkerSize',10, ...
        'MarkerFaceColor','k');

    %% Plot predicted trajectory
    plot(predictedX,predictedY,'--o', ...
        'LineWidth',1.5, ...
        'MarkerSize',4);

    %% Label
    text( ...
        x+0.4, ...
        y+0.4, ...
        sprintf('T%d | %s', ...
        objects(i).id, ...
        objects(i).type), ...
        'FontWeight','bold');

    %% Final predicted point
    plot( ...
        predictedX(end), ...
        predictedY(end), ...
        's', ...
        'MarkerSize',9, ...
        'LineWidth',1.5);
end

%% Formatting
xlim([-10 55]);
ylim([-12 12]);

axis equal;

xlabel('Longitudinal Position X (m)');
ylabel('Lateral Position Y (m)');

title(sprintf( ...
    'M4-A: Short-Term Motion Prediction | Horizon = %.1f s', ...
    predictionHorizon));

%% Print prediction table
fprintf('\n');
fprintf('========================================\n');
fprintf(' SIH 26037 - M4-A PREDICTIONS\n');
fprintf('========================================\n');

for i = 1:numObjects

    finalPrediction = predictions{i}(end,:);

    fprintf( ...
        '%-15s | T%d | Final predicted position: X = %6.2f m, Y = %6.2f m\n', ...
        objects(i).type, ...
        objects(i).id, ...
        finalPrediction(1), ...
        finalPrediction(2));
end

fprintf('========================================\n');

disp("M4-A motion prediction completed successfully.");