function ego = createTestEgoState()
%CREATETESTEGOSTATE Create a standard synthetic ego state for testing.

ego.position = [0 0];
ego.velocity = [8 0];
ego.heading = 0;
ego.speed = 8;
ego.timestamp = 0;
end