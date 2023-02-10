function exampleCommandBuildWorld(coordinator)

%CommandBuildWorld Construct the world used for visualization and collision detection
%   This command constructs the environment, consisting of the workstation,
%   belts, parts to be moved, and obstacles. The robot is constructed
%   separately. These objects are created using collision primitives and
%   placed in the right spots. In the workflow, this world is used during
%   trajectory generation/optimization for collision-checking, and during
%   the visualization step.
%

% Copyright 2020 The MathWorks, Inc.

    % Construct the Workstation (only for visualization)
    bench = collisionBox(0.5, 0.7, 0.05);
    belt1 = collisionBox(1.3, 0.4, 0.05);
    belt2 = collisionBox(1.3, 0.4, 0.05);

    TBench = trvec2tform([0.4 0 0.2]);
    TBelt1 = trvec2tform([0 -0.6 0.2]);
    TBelt2 = trvec2tform([0 0.6 0.2]);

    bench.Pose = TBench;
    belt1.Pose = TBelt1;
    belt2.Pose = TBelt2;
    
    coordinator.World = {bench, belt1, belt2};
    
    obs1 = collisionSphere(0.13);
    Tobs1 = trvec2tform([0.4 0.38 0.4]);
    obs1.Pose = Tobs1;
    
    obs2 = collisionSphere(0.13);
    Tobs2 = trvec2tform([0.4 -0.38 0.4]);
    obs2.Pose = Tobs2;

    coordinator.Obstacles = {obs1, obs2};    

    % Add the parts, which are only used for visualization and
    % simulation. A separate tool ensures that when a part is
    % gripped, it is included in the collision detection stage of
    % the trajectory optimization workflow.
    box2 = collisionBox(0.06, 0.06, 0.1);
    box3 = collisionBox(0.06, 0.06, 0.1);
    box1 = collisionBox(0.06, 0.06, 0.1);

    % Move the parts into position
    TBox2 = trvec2tform([0.5 -0.15 0.26]);
    TBox3 = trvec2tform([0.52 0 0.26]);
    TBox1 = trvec2tform([0.4 -0.1 0.26]);

    box2.Pose = TBox2;
    box3.Pose = TBox3;
    box1.Pose = TBox1;

    % Set the part mesh and color
    part1.mesh = box2;
    part2.mesh = box3;
    part3.mesh = box1;

    part1.color = 'y';
    part2.color = 'y';
    part3.color = 'g';

    part1.centerPoint = tform2trvec(part1.mesh.Pose);
    part2.centerPoint = tform2trvec(part2.mesh.Pose);
    part3.centerPoint = tform2trvec(part3.mesh.Pose);

    part1.plot = [];
    part2.plot = [];
    part3.plot = [];

    coordinator.Parts = {part1, part2, part3};

    % Visualize world and parts
    visualizeWorld(coordinator)
    visualizeParts(coordinator)

   % Trigger Stateflow chart Event
   coordinator.FlowChart.worldBuilt;
end

