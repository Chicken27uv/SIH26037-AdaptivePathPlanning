function validateInterface()
%VALIDATEINTERFACE Verify the basic shared project data contracts.

ego = createTestEgoState();
objects = createTestObjects();
trajectory = createTestTrajectory();

%% Validate ego state
assert(isfield(ego, "position"));
assert(isfield(ego, "velocity"));
assert(isfield(ego, "heading"));
assert(isfield(ego, "speed"));
assert(isfield(ego, "timestamp"));

assert(numel(ego.position) == 2);
assert(numel(ego.velocity) == 2);

%% Validate objects
requiredObjectFields = ...
    ["id","class","position","velocity","heading","timestamp"];

for k = 1:numel(objects)
    for f = requiredObjectFields
        assert(isfield(objects(k), f), ...
            "Missing object field: " + f);
    end

    assert(numel(objects(k).position) == 2);
    assert(numel(objects(k).velocity) == 2);
end

%% Validate trajectory
assert(size(trajectory,2) == 2);

disp("Common interface validation PASSED.");
end