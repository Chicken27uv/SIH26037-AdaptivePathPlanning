%% SIH 26037
% Module M3 - Perception and Object Tracking
% Developer: Daniel
%
% Stage M3-E:
% Final class-aware multi-object tracking
% with improved velocity estimation and metrics

clc;
clear;
close all;

rng(7);

%% Simulation settings
dt = 0.1;
simTime = 12;
time = 0:dt:simTime;

%% Detection settings
detectionProbability = 0.92;
positionNoiseStd = 0.45;

%% Tracker settings
associationThreshold = 2.5;
maxMissedFrames = 5;
velocitySmoothing = 0.85;

%% Ego vehicle
ego.x = 0;
ego.y = 0;
ego.vx = 4;
ego.vy = 0;

%% Ground-truth actors
actors(1).id = 1;
actors(1).type = "Car";
actors(1).state = [20, 2.5, -1.0, 0];

actors(2).id = 2;
actors(2).type = "Pedestrian";
actors(2).state = [12, 7, 0, -0.8];

actors(3).id = 3;
actors(3).type = "Auto-rickshaw";
actors(3).state = [8, -4, 2.0, 0.25];

actors(4).id = 4;
actors(4).type = "Two-wheeler";
actors(4).state = [-5, -2, 5.0, 0.15];

actors(5).id = 5;
actors(5).type = "Cattle";
actors(5).state = [25, -7, -0.15, 0.7];

numActors = length(actors);

%% Track structure
tracks = struct( ...
    'trackID', {}, ...
    'position', {}, ...
    'velocity', {}, ...
    'age', {}, ...
    'missedFrames', {}, ...
    'history', {}, ...
    'type', {}, ...
    'trueActorID', {});

nextTrackID = 1;

%% Metrics storage
totalDetections = 0;
createdTracks = 0;
maintainedDuringMiss = 0;

positionErrors = [];
velocityErrors = [];

activeTrackCount = zeros(size(time));

actorTrackIDs = cell(1,numActors);

%% Figure
figure( ...
    'Name','SIH 26037 - M3-E Final Tracking', ...
    'NumberTitle','off');

%% Main simulation loop
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

    %% -------------------------------------------------
    % STEP 1: Generate noisy detections
    %% -------------------------------------------------

    detections = [];

    for i = 1:numActors

        % Update ground truth
        actors(i).state(1) = ...
            actors(i).state(1) + actors(i).state(3)*dt;

        actors(i).state(2) = ...
            actors(i).state(2) + actors(i).state(4)*dt;

        trueX = actors(i).state(1);
        trueY = actors(i).state(2);

        %% Plot ground truth
        plot(trueX,trueY,'bo', ...
            'MarkerSize',7);

        %% Generate detection
        if rand <= detectionProbability

            measuredX = ...
                trueX + positionNoiseStd*randn;

            measuredY = ...
                trueY + positionNoiseStd*randn;

            detection.position = [measuredX measuredY];
            detection.type = actors(i).type;
            detection.trueActorID = actors(i).id;

            if isempty(detections)
                detections = detection;
            else
                detections(end+1) = detection;
            end

            totalDetections = totalDetections + 1;

            %% Plot raw detection
            plot(measuredX,measuredY,'rx', ...
                'MarkerSize',9, ...
                'LineWidth',1.5);
        end
    end

    %% -------------------------------------------------
    % STEP 2: Predict existing tracks
    %% -------------------------------------------------

    for t = 1:length(tracks)

        tracks(t).position = ...
            tracks(t).position + tracks(t).velocity*dt;
    end

    %% -------------------------------------------------
    % STEP 3: Class-aware nearest-neighbour association
    %% -------------------------------------------------

    assignedDetection = false(1,length(detections));
    assignedTrack = false(1,length(tracks));

    if ~isempty(tracks) && ~isempty(detections)

        distanceMatrix = inf(length(tracks),length(detections));

        for t = 1:length(tracks)

            for d = 1:length(detections)

                % Only allow association if class is the same
                if tracks(t).type == detections(d).type

                    distanceMatrix(t,d) = norm( ...
                        tracks(t).position - ...
                        detections(d).position);
                end
            end
        end

        %% Greedy association
        while true

            [minDistance,linearIndex] = ...
                min(distanceMatrix(:));

            if isempty(minDistance) || ...
               isinf(minDistance) || ...
               minDistance > associationThreshold

                break;
            end

            [trackIdx,detIdx] = ...
                ind2sub(size(distanceMatrix),linearIndex);

            %% Recover previous corrected position
            previousCorrectedPosition = ...
                tracks(trackIdx).position - ...
                tracks(trackIdx).velocity*dt;

            %% Estimate measured velocity
            measuredVelocity = ...
                (detections(detIdx).position - ...
                 previousCorrectedPosition) / dt;

            %% Smooth velocity estimate
            tracks(trackIdx).velocity = ...
                velocitySmoothing*tracks(trackIdx).velocity + ...
                (1-velocitySmoothing)*measuredVelocity;

            %% Correct track position
            tracks(trackIdx).position = ...
                detections(detIdx).position;

            tracks(trackIdx).missedFrames = 0;
            tracks(trackIdx).age = ...
                tracks(trackIdx).age + 1;

            tracks(trackIdx).type = ...
                detections(detIdx).type;

            tracks(trackIdx).trueActorID = ...
                detections(detIdx).trueActorID;

            tracks(trackIdx).history(end+1,:) = ...
                tracks(trackIdx).position;

            assignedDetection(detIdx) = true;
            assignedTrack(trackIdx) = true;

            %% Prevent reuse in same frame
            distanceMatrix(trackIdx,:) = inf;
            distanceMatrix(:,detIdx) = inf;

            %% Tracking error metrics
            actorID = detections(detIdx).trueActorID;

            truePosition = ...
                actors(actorID).state(1:2);

            trueVelocity = ...
                actors(actorID).state(3:4);

            positionErrors(end+1) = norm( ...
                tracks(trackIdx).position - ...
                truePosition);

            velocityErrors(end+1) = norm( ...
                tracks(trackIdx).velocity - ...
                trueVelocity);

            actorTrackIDs{actorID}(end+1) = ...
                tracks(trackIdx).trackID;
        end
    end

    %% -------------------------------------------------
    % STEP 4: Handle missed tracks
    %% -------------------------------------------------

    for t = 1:length(tracks)

        if ~assignedTrack(t)

            tracks(t).missedFrames = ...
                tracks(t).missedFrames + 1;

            tracks(t).age = ...
                tracks(t).age + 1;

            tracks(t).history(end+1,:) = ...
                tracks(t).position;

            maintainedDuringMiss = ...
                maintainedDuringMiss + 1;
        end
    end

    %% -------------------------------------------------
    % STEP 5: Create new tracks
    %% -------------------------------------------------

    for d = 1:length(detections)

        if ~assignedDetection(d)

            newTrack.trackID = nextTrackID;
            newTrack.position = detections(d).position;
            newTrack.velocity = [0 0];
            newTrack.age = 1;
            newTrack.missedFrames = 0;
            newTrack.history = detections(d).position;
            newTrack.type = detections(d).type;
            newTrack.trueActorID = detections(d).trueActorID;

            tracks(end+1) = newTrack;

            actorID = detections(d).trueActorID;

            actorTrackIDs{actorID}(end+1) = ...
                nextTrackID;

            nextTrackID = ...
                nextTrackID + 1;

            createdTracks = ...
                createdTracks + 1;
        end
    end

    %% -------------------------------------------------
    % STEP 6: Remove stale tracks
    %% -------------------------------------------------

    if ~isempty(tracks)

        keepTrack = true(1,length(tracks));

        for t = 1:length(tracks)

            if tracks(t).missedFrames > maxMissedFrames
                keepTrack(t) = false;
            end
        end

        tracks = tracks(keepTrack);
    end

    %% Store track count
    activeTrackCount(k) = length(tracks);

    %% -------------------------------------------------
    % STEP 7: Plot active tracks
    %% -------------------------------------------------

    for t = 1:length(tracks)

        x = tracks(t).position(1);
        y = tracks(t).position(2);

        %% Track position
        plot(x,y,'md', ...
            'MarkerSize',10, ...
            'LineWidth',2);

        %% Track history
        if size(tracks(t).history,1) >= 2

            plot( ...
                tracks(t).history(:,1), ...
                tracks(t).history(:,2), ...
                'm-', ...
                'LineWidth',1);
        end

        %% Velocity arrow
        quiver( ...
            x, ...
            y, ...
            tracks(t).velocity(1), ...
            tracks(t).velocity(2), ...
            0.5, ...
            'LineWidth',1.2);

        %% Label
        label = sprintf( ...
            'T%d | %s', ...
            tracks(t).trackID, ...
            tracks(t).type);

        if tracks(t).missedFrames > 0

            label = sprintf( ...
                '%s | predicted', ...
                label);
        end

        text( ...
            x+0.4, ...
            y+0.4, ...
            label, ...
            'FontWeight','bold');
    end

    %% Figure formatting
    xlim([-10 55]);
    ylim([-12 12]);

    axis equal;

    xlabel('Longitudinal Position X (m)');
    ylabel('Lateral Position Y (m)');

    title(sprintf( ...
        'M3-E Final Tracking | Time = %.1f s | Active Tracks = %d', ...
        time(k),length(tracks)));

    %% Legend
    h1 = plot(nan,nan,'bo');
    h2 = plot(nan,nan,'rx','LineWidth',1.5);
    h3 = plot(nan,nan,'md','LineWidth',2);
    h4 = plot(nan,nan,'ks','MarkerFaceColor','k');

    legend( ...
        [h1 h2 h3 h4], ...
        {'Ground Truth', ...
         'Detection', ...
         'Tracked Estimate', ...
         'Ego Vehicle'}, ...
        'Location','northeastoutside');

    drawnow;
end

%% -------------------------------------------------
% FINAL METRICS
%% -------------------------------------------------

positionRMSE = ...
    sqrt(mean(positionErrors.^2));

velocityRMSE = ...
    sqrt(mean(velocityErrors.^2));

%% Track fragmentation analysis
totalFragments = 0;

fprintf('\n');
fprintf('Track IDs observed per actor:\n');

for i = 1:numActors

    uniqueIDs = ...
        unique(actorTrackIDs{i});

    fprintf('%-15s : ',actors(i).type);

    fprintf('%d ',uniqueIDs);

    fprintf('\n');

    if ~isempty(uniqueIDs)

        totalFragments = ...
            totalFragments + ...
            max(length(uniqueIDs)-1,0);
    end
end

%% Final summary
fprintf('\n');
fprintf('========================================\n');
fprintf(' SIH 26037 - M3-E FINAL RESULTS\n');
fprintf('========================================\n');

fprintf('Total detections          : %d\n', ...
    totalDetections);

fprintf('Tracks created            : %d\n', ...
    createdTracks);

fprintf('Active tracks at end      : %d\n', ...
    length(tracks));

fprintf('Missed frames predicted   : %d\n', ...
    maintainedDuringMiss);

fprintf('Tracking position RMSE    : %.3f m\n', ...
    positionRMSE);

fprintf('Tracking velocity RMSE    : %.3f m/s\n', ...
    velocityRMSE);

fprintf('Track fragmentations      : %d\n', ...
    totalFragments);

fprintf('========================================\n');

%% -------------------------------------------------
% ACTIVE TRACK COUNT PLOT
%% -------------------------------------------------

figure( ...
    'Name','M3-E Active Track Count', ...
    'NumberTitle','off');

plot(time,activeTrackCount, ...
    'LineWidth',1.5);

grid on;

xlabel('Time (s)');
ylabel('Number of Active Tracks');

title('M3-E Active Tracks Over Time');

yline(numActors,'--','Expected Actors');

%% -------------------------------------------------
% ERROR METRIC PLOT
%% -------------------------------------------------

figure( ...
    'Name','M3-E Tracking Error Metrics', ...
    'NumberTitle','off');

metricValues = [
    positionRMSE
    velocityRMSE
];

bar(metricValues);

set(gca, ...
    'XTick',1:2, ...
    'XTickLabel', ...
    {'Position RMSE (m)', ...
     'Velocity RMSE (m/s)'});

ylabel('Error');
title('M3-E Tracking Performance');
grid on;

disp("M3-E final tracking simulation completed successfully.");