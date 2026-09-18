function objects = createTestObjects()
%CREATETESTOBJECTS Create synthetic tracked objects for module testing.

objects(1).id = 1;
objects(1).class = "pedestrian";
objects(1).position = [18 2];
objects(1).velocity = [0.5 0];
objects(1).heading = 0;
objects(1).timestamp = 0;

objects(2).id = 2;
objects(2).class = "auto_rickshaw";
objects(2).position = [30 -2];
objects(2).velocity = [5 0];
objects(2).heading = 0;
objects(2).timestamp = 0;

objects(3).id = 3;
objects(3).class = "animal";
objects(3).position = [45 1];
objects(3).velocity = [-1 0];
objects(3).heading = pi;
objects(3).timestamp = 0;
end