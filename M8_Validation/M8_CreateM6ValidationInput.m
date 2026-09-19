function input = M8_CreateM6ValidationInput(name)

input.map = M6_CreateMap;

input.startPose = [5 5 0];
input.goalPose = [55 25 0];

input.decision.behavior = "AVOID";
input.decision.replanRequired = true;

switch name

    case "VillageRoad"
        input.predictedObjects(1).id = 1;
        input.predictedObjects(1).class = "pedestrian";
        input.predictedObjects(1).position = [24 9];
        input.predictedObjects(1).velocity = [0 3];
        input.predictedObjects(1).heading = pi/2;
        input.predictedObjects(1).timestamp = 0;

    case "UrbanIntersection"
        input.predictedObjects(1).id = 2;
        input.predictedObjects(1).class = "auto-rickshaw";
        input.predictedObjects(1).position = [23 10];
        input.predictedObjects(1).velocity = [0 2.5];
        input.predictedObjects(1).heading = pi/2;
        input.predictedObjects(1).timestamp = 0;

    case "HighwayMerge"
        input.predictedObjects(1).id = 3;
        input.predictedObjects(1).class = "vehicle";
        input.predictedObjects(1).position = [23 14];
        input.predictedObjects(1).velocity = [1 0.5];
        input.predictedObjects(1).heading = atan2(0.5,1);
        input.predictedObjects(1).timestamp = 0;

    case "MarketArea"
        input.predictedObjects(1).id = 4;
        input.predictedObjects(1).class = "pedestrian";
        input.predictedObjects(1).position = [25 9];
        input.predictedObjects(1).velocity = [0 2.5];
        input.predictedObjects(1).heading = pi/2;
        input.predictedObjects(1).timestamp = 0;

    case "CattleCrossing"
        input.predictedObjects(1).id = 5;
        input.predictedObjects(1).class = "cattle";
        input.predictedObjects(1).position = [24 9];
        input.predictedObjects(1).velocity = [0 3];
        input.predictedObjects(1).heading = pi/2;
        input.predictedObjects(1).timestamp = 0;

    otherwise
        error("Unknown scenario: %s",name)

end

end