%% SIH 26037
% Module M2 - Multi-Sensor Simulation and Fusion
% Stage M2-B: Camera-like detections
% Developer: Daniel (supporting M2)

clc;
clear;
close all;

rng(10);

%% Simulation settings
dt = 0.1;
simTime = 10;
time = 0:dt:simTime;

%% Camera settings
cameraRange = 30;              % metres
cameraHalfFOVdeg = 18;         % half-angle of camera FOV
cameraDetectionProbability = 0.90;
cameraNoiseStd = 0.35;         % metres

%% Ego vehicle
ego.x = 0;
ego.y = 0;
ego.vx = 3.5;
ego.vy = 0;

%% Actors
actors(1).id = 1;
actors(1).type = "Car";
actors(1).state = [22, 2.5, -1.0, 0];

actors(2).id = 2;
actors(2).type = "Pedestrian";
actors(2).state = [14, 6.5, 0, -0.7];

actors(3).id = 3;
actors(3).type = "Auto-rickshaw";
actors(3).state = [10, -4.5, 1.8, 0.2];

actors(4).id = 4;
actors(4).type = "Two-wheeler";
actors(4).state = [-4, -2.0, 4.5, 0.1];

actors(5).id = 5;
actors(5).type = "Cattle";
actors(5).state = [27, -7.0, -0.1, 0.65];

%% Metrics
visibleOpportunities = 0;
successfulDetections = 0;
cameraErrors = [];

%% Figure
figure( ...
    'Name','SIH 26037 - M2-B Camera Detections', ...
    'NumberTitle','off');

%% Simulation loop
for k = 1:length(time)

    cla;
    hold on;
    grid on;

    %% Draw road
    patch([-10 55 55 -10], ...
          [-8 -8 8 8], ...
          [0.85 0.85 0.85], ...
          'EdgeColor','none');

    %% Update ego
    ego.x = ego.x + ego.vx*dt;
    ego.y = ego.y + ego.vy*dt;

    plot(ego.x,ego.y,'ks', ...
        'MarkerSize',12, ...
        'MarkerFaceColor','k');

    text(ego.x,ego.y-1, ...
        'EGO', ...
        'HorizontalAlignment','center', ...
        'FontWeight','bold');

    %% Draw camera FOV
    fovAngle = deg2rad(cameraHalfFOVdeg);

    leftX = ego.x + cameraRange*cos(fovAngle);
    leftY = ego.y + cameraRange*sin(fovAngle);

    rightX = ego.x + cameraRange*cos(fovAngle);
    rightY = ego.y - cameraRange*sin(fovAngle);

    plot([ego.x leftX], ...
         [ego.y leftY], ...
         'k--');

    plot([ego.x rightX], ...
         [ego.y rightY], ...
         'k--');

    plot([leftX rightX], ...
         [leftY rightY], ...
         'k:');

    %% Update actors
    for i = 1:length(actors)

        actors(i).state(1) = ...
            actors(i).state(1) + actors(i).state(3)*dt;

        actors(i).state(2) = ...
            actors(i).state(2) + actors(i).state(4)*dt;

        trueX = actors(i).state(1);
        trueY = actors(i).state(2);

        %% Plot ground truth
        plot(trueX,trueY,'bo', ...
            'MarkerSize',8, ...
            'LineWidth',1.5);

        text(trueX+0.3,trueY+0.4, ...
            sprintf('%s',actors(i).type));

        %% Relative position wrt ego
        dx = trueX - ego.x;
        dy = trueY - ego.y;

        range = hypot(dx,dy);

        angleToActor = atan2(dy,dx);

        %% Check if actor is in forward camera FOV
        visibleToCamera = ...
            dx > 0 && ...
            range <= cameraRange && ...
            abs(angleToActor) <= fovAngle;

        if visibleToCamera

            visibleOpportunities = ...
                visibleOpportunities + 1;

            %% Detection probability
            if rand <= cameraDetectionProbability

                measuredX = ...
                    trueX + cameraNoiseStd*randn;

                measuredY = ...
                    trueY + cameraNoiseStd*randn;

                successfulDetections = ...
                    successfulDetections + 1;

                err = hypot( ...
                    measuredX-trueX, ...
                    measuredY-trueY);

                cameraErrors(end+1) = err;

                %% Plot camera detection
                plot(measuredX,measuredY,'rx', ...
                    'MarkerSize',10, ...
                    'LineWidth',2);

                plot([trueX measuredX], ...
                     [trueY measuredY], ...
                     'k:');
            end
        end
    end

    %% Formatting
    xlim([-10 55]);
    ylim([-12 12]);

    axis equal;

    xlabel('Longitudinal Position X (m)');
    ylabel('Lateral Position Y (m)');

    title(sprintf( ...
        'M2-B Camera-like Detections | Time = %.1f s', ...
        time(k)));

    %% Legend
    h1 = plot(nan,nan,'bo');
    h2 = plot(nan,nan,'rx','LineWidth',2);
    h3 = plot(nan,nan,'ks','MarkerFaceColor','k');

    legend( ...
        [h1 h2 h3], ...
        {'Ground Truth', ...
         'Camera Detection', ...
         'Ego Vehicle'}, ...
        'Location','northeastoutside');

    drawnow;
end

%% Metrics
if visibleOpportunities > 0

    cameraDetectionRate = ...
        successfulDetections / visibleOpportunities;

else
    cameraDetectionRate = 0;
end

if ~isempty(cameraErrors)

    cameraMeanError = ...
        mean(cameraErrors);

    cameraRMSE = ...
        sqrt(mean(cameraErrors.^2));

else
    cameraMeanError = NaN;
    cameraRMSE = NaN;
end

%% Results
fprintf('\n');
fprintf('============================================\n');
fprintf(' SIH 26037 - M2-B CAMERA RESULTS\n');
fprintf('============================================\n');

fprintf('Visible opportunities : %d\n', ...
    visibleOpportunities);

fprintf('Camera detections     : %d\n', ...
    successfulDetections);

fprintf('Detection rate        : %.2f %%\n', ...
    cameraDetectionRate*100);

fprintf('Mean position error   : %.3f m\n', ...
    cameraMeanError);

fprintf('Position RMSE         : %.3f m\n', ...
    cameraRMSE);

fprintf('============================================\n');

disp("M2-B camera detection simulation completed successfully.");