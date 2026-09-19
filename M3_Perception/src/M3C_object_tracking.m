%% SIH 26037
% Module M3 - Perception and Object Tracking
% Developer: Daniel
%
% Stage M3-C:
% Lightweight multi-object tracking without
% Sensor Fusion and Tracking Toolbox
%
% Method:
% 1. Simulate noisy detections
% 2. Associate detections to existing tracks
% 3. Estimate velocity
% 4. Maintain persistent IDs
% 5. Survive short missed-detection periods

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
associationThreshold = 2.5;   % metres
maxMissedFrames = 5;
velocitySmoothing = 0.65;

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

%% Track structure
tracks = struct( ...
    'trackID', {}, ...
    'position', {}, ...
    'velocity', {}, ...
    'age', {}, ...
    'missedFrames', {}, ...
    'history', {}, ...
    'type', {});

nextTrackID = 1;

%% Metrics
totalDetections = 0;
createdTracks = 0;
maintainedDuringMiss = 0;

%% Figure
figure( ...
    'Name','SIH 26037 - M3-C Object Tracking', ...
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

    for i = 1:length(actors)

        actors(i).state(1) = ...
            actors(i).state(1) + actors(i).state(3)*dt;

        actors(i).state(2) = ...
            actors(i).state(2) + actors(i).state(4)*dt;

        trueX = actors(i).state(1);
        trueY = actors(i).state(2);

        %% Plot ground truth
        plot(trueX,trueY,'bo', ...
            'MarkerSize',7);

        %% Generate detection probabilistically
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
    % STEP 2: Predict current track positions
    %% -------------------------------------------------

    for t = 1:length(tracks)
        tracks(t).position = ...
            tracks(t).position + tracks(t).velocity*dt;
    end

    %% -------------------------------------------------
    % STEP 3: Associate detections to tracks
    %% -------------------------------------------------

    assignedDetection = false(1,length(detections));
    assignedTrack = false(1,length(tracks));

    if ~isempty(tracks) && ~isempty(detections)

        distanceMatrix = inf(length(tracks),length(detections));

        for t = 1:length(tracks)
            for d = 1:length(detections)

                distanceMatrix(t,d) = norm( ...
                    tracks(t).position - ...
                    detections(d).position);
            end
        end

        %% Greedy nearest-neighbour association
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

            %% Update velocity
            measuredVelocity = ...
                (detections(detIdx).position - ...
                 tracks(trackIdx).position) / dt;

            tracks(trackIdx).velocity = ...
                velocitySmoothing*tracks(trackIdx).velocity + ...
                (1-velocitySmoothing)*measuredVelocity;

            %% Update position
            tracks(trackIdx).position = ...
                detections(detIdx).position;

            tracks(trackIdx).missedFrames = 0;
            tracks(trackIdx).age = tracks(trackIdx).age + 1;
            tracks(trackIdx).type = detections(detIdx).type;

            tracks(trackIdx).history(end+1,:) = ...
                tracks(trackIdx).position;

            assignedDetection(detIdx) = true;
            assignedTrack(trackIdx) = true;

            distanceMatrix(trackIdx,:) = inf;
            distanceMatrix(:,detIdx) = inf;
        end
    end

    %% -------------------------------------------------
    % STEP 4: Handle missed tracks
    %% -------------------------------------------------

    for t = 1:length(tracks)

        if ~assignedTrack(t)

            tracks(t).missedFrames = ...
                tracks(t).missedFrames + 1;

            tracks(t).age = tracks(t).age + 1;

            tracks(t).history(end+1,:) = ...
                tracks(t).position;

            if tracks(t).missedFrames > 0
                maintainedDuringMiss = ...
                    maintainedDuringMiss + 1;
            end
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

            tracks(end+1) = newTrack;

            nextTrackID = nextTrackID + 1;
            createdTracks = createdTracks + 1;
        end
    end

    %% -------------------------------------------------
    % STEP 6: Delete stale tracks
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

    %% -------------------------------------------------
    % STEP 7: Plot tracked objects
    %% -------------------------------------------------

    for t = 1:length(tracks)

        x = tracks(t).position(1);
        y = tracks(t).position(2);

        %% Estimated track position
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

        %% Velocity vector
        quiver( ...
            x, ...
            y, ...
            tracks(t).velocity(1), ...
            tracks(t).velocity(2), ...
            0.5, ...
            'LineWidth',1.2);

        %% Track label
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
        'M3-C Multi-Object Tracking | Time = %.1f s | Active Tracks = %d', ...
        time(k), ...
        length(tracks)));

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

%% Final summary
fprintf('\n');
fprintf('========================================\n');
fprintf(' SIH 26037 - M3-C TRACKING RESULTS\n');
fprintf('========================================\n');

fprintf('Total detections       : %d\n', ...
    totalDetections);

fprintf('Tracks created         : %d\n', ...
    createdTracks);

fprintf('Active tracks at end   : %d\n', ...
    length(tracks));

fprintf('Predicted missed frames: %d\n', ...
    maintainedDuringMiss);

fprintf('========================================\n');

disp("M3-C tracking simulation completed successfully.");