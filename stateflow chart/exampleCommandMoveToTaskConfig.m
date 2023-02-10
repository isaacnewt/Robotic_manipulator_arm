function exampleCommandMoveToTaskConfig(coordinator, taskConfig, tolerance, avoidCollisions)

%CommandMoveToTaskConfig Move the manipulator to a task-space position
%   This command moves the manipulator from its current pose to a
%   desired task-space pose. The move is executed using a nonlinear model
%   predictive controller (nlmpc) to generate a collision-free reference trajectory.
%   The trajectory is simulated using a model of the robot under
%   joint-space motion control. This also updates the visualization. 
%
% Copyright 2020 The MathWorks, Inc.

        %   Execute the command, which simulates robot motion from the current to the target pose
        %   To execute this motion, the command calls a helper
        %   function, exampleHelperPlanExecuteTrajectoryPickPlace, that
        %   plans an optimized reference trajectory using a nonlinear
        %   MPC solver and then simulates the motion of the robot under
        %   joint-space position control. The helper function also
        %   ensures that the visualization is updated accordingly.

        % Execute closed-loop trajectory optimization and control using
        % model predictive control
        mpcTimeStep = 0.6;
        [positions, velocities, accelerations, timestamp, success] = exampleHelperPlanExecuteTrajectoryPickPlace(coordinator.Robot, mpcTimeStep,  coordinator.Obstacles, coordinator.RobotEndEffector, coordinator.CurrentRobotJConfig, taskConfig, tolerance, avoidCollisions);
        if success==0
            error('Cannot compute motion to reach desired task configuration. Aborting...')
        end
        
        %% Execute the trajectory using low-fidelity simulation
        targetStates = [positions;velocities;accelerations]'; 
        targetTime = timestamp;
        initState = [positions(:,1);velocities(:,1)]';
        trajTimes = targetTime(1):coordinator.TimeStep:targetTime(end);

        [~,robotStates] = ode15s(@(t,state) exampleHelperTimeBasedStateInputsPickPlace(coordinator.MotionModel, targetTime, targetStates, t, state), trajTimes, initState);

        %% Visualize trajectory
        % Uncomment below to display all successful outputs
        % disp('Executing collision-free trajectory...')
        visualizePath(coordinator,positions);
        visualizeRobot(coordinator, robotStates, trajTimes);
        
        % Deleta path on plot
        coordinator.PathHandle.Visible = 'off';

        % Update current robot configuration
        coordinator.CurrentRobotJConfig = positions(:,end)';
        coordinator.CurrentRobotTaskConfig = getTransform(coordinator.Robot, coordinator.CurrentRobotJConfig, coordinator.RobotEndEffector); 

        % Trigger Stateflow chart Event
        coordinator.FlowChart.taskConfigReached; 
end