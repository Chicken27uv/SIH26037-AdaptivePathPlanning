function input = M5_CreateTestInput(scenario)

input.ego.position = [0 0];
input.ego.velocity = [5 0];
input.ego.heading = 0;

switch lower(string(scenario))

    case "empty"
        input.objects = struct.empty;

    case "slowvehicle"
        input.objects = M5_CreateObject( ...
            "vehicle",[12 0],[2 0],[12 0;14 0;16 0;18 0]);

    case "pedestrian"
        input.objects = M5_CreateObject( ...
            "pedestrian",[8 -3],[0 1],[8 -3;8 -1;8 1;8 3]);

    case "cattle"
        input.objects = M5_CreateObject( ...
            "cattle",[7 0],[-0.5 0],[7 0;6.5 0;6 0;5.5 0]);

    case "emergency"
        input.objects = M5_CreateObject( ...
            "cattle",[2 0],[-1 0],[2 0;1.5 0;1 0;0.5 0]);

    case "multi"
        input.objects(1) = M5_CreateObject( ...
            "vehicle",[14 0],[1 0],[14 0;15 0;16 0;17 0]);

        input.objects(2) = M5_CreateObject( ...
            "pedestrian",[9 -2],[0 1],[9 -2;9 -0.5;9 1;9 2.5]);

        input.objects(3) = M5_CreateObject( ...
            "cattle",[6 5],[0 -0.5],[6 5;6 4.5;6 4;6 3.5]);

    otherwise
        error("Unknown M5 test scenario.")

end

end

function object = M5_CreateObject(class,position,velocity,predictedTrajectory)

object.class = class;
object.position = position;
object.velocity = velocity;
object.predictedTrajectory = predictedTrajectory;

end