classdef exampleHelperCoordinatorPickPlace < handle
% This class is for internal use and may be removed in a future release
%
%exampleHelperCoordinatorPickPlace Class used to run the Stateflow chart
%and hold all pick and place runtime data
%   This class is used to control the pick-and-place workflow execution.
%   The class serves two main purposes:
%      1. It handles visualization for the
%         workflow 
%      2. It stores all data about the current pick and place job
% Copyright 2020 The MathWorks, Inc.

    properties         
        FlowChart
        Robot
        World = {};
        Parts = {};
        Obstacles = {};
        DetectedParts = {};
        RobotEndEffector
        CurrentRobotJConfig
        CurrentRobotTaskConfig
        NextPart = 0;
        PartOnRobot = 0;
        HomeRobotTaskConfig 
        PlacingPose
        GraspPose
        Figure
        TimeStep
        MotionModel
        NumJoints
        NumDetectionRuns = 0;
        PathHandle
    end
    
    methods
        function obj = exampleHelperCoordinatorPickPlace(robot, currentRobotJConfig, robotEndEffector)
            obj.Robot = robot;            
            obj.CurrentRobotJConfig = currentRobotJConfig;
            obj.RobotEndEffector = robotEndEffector;
            obj.CurrentRobotTaskConfig = getTransform(obj.Robot, obj.CurrentRobotJConfig, obj.RobotEndEffector);
            obj.TimeStep = 0.1; % Visualization time step
            obj.MotionModel = jointSpaceMotionModel('RigidBodyTree', obj.Robot);
            obj.NumJoints = numel(obj.CurrentRobotJConfig);
                    
            % Initialize visualization
            obj.Figure = interactiveRigidBodyTree(obj.Robot,'ShowMarker',false, 'Frames', 'off'); 
            obj.Figure.Configuration = obj.CurrentRobotJConfig;
            obj.Figure.ShowMarker = false;
            hold on
            axis([-1 1 -1 1 -0.1 1.5]);
            view(58,25);            
        end
        
    
       function visualizeWorld(obj)
           try
            bench = obj.World{1};
            belt1 = obj.World{2};
            belt2 = obj.World{3};

            % Render world
            [~, p1] = show(bench);
            [~, p2] = show(belt1);
            [~, p3] = show(belt2);

            p1.FaceColor = [1 0.5 0];
            p1.FaceAlpha = 1.0;
            p1.LineStyle = 'none';

            p2.FaceColor = [128,128,128]/255;
            p2.FaceAlpha = 1.0;
            p2.LineStyle = 'none';

            p3.FaceColor = [128,128,128]/255;
            p3.FaceAlpha = 1.0;  
            p3.LineStyle = 'none';
            
            % Visualize obstacles
            for i=1:numel(obj.Obstacles)
                [~, obs] = show(obj.Obstacles{i});
                obs.LineStyle = 'none';
                obs.FaceColor = 'b';
            end

            drawnow;
           catch
           end
       end


        function visualizeParts(obj)
            for i = 1:length(obj.Parts)
                tempPose = [0,0,0]; % to set transformation reference
                correctPose = obj.Parts{i}.mesh.Pose;
                obj.Parts{i}.mesh.Pose = trvec2tform(tempPose);
                [~, obj.Parts{i}.plot] = show(obj.Parts{i}.mesh);
                obj.Parts{i}.plot.LineStyle = 'none'; 
                obj.Parts{i}.plotHandle = hgtransform;
                obj.Parts{i}.plot.Parent = obj.Parts{i}.plotHandle;
                obj.Parts{i}.mesh.Pose = correctPose;
                obj.Parts{i}.plotHandle.Matrix = obj.Parts{i}.mesh.Pose;
                obj.Parts{i}.plot.FaceColor = obj.Parts{i}.color; 
            end
            drawnow;
        end
        

        function visualizeRobot(obj, robotStates, trajTimes)
            % Visualize robot motion           
            for k = 1:length(trajTimes)
                configNow = robotStates(k,1:obj.NumJoints);
                obj.Figure.Configuration = configNow;
                obj.Figure.ShowMarker = false;
                % Update current robot configuration
                obj.CurrentRobotJConfig = configNow;
                obj.CurrentRobotTaskConfig = getTransform(obj.Robot, obj.CurrentRobotJConfig, obj.RobotEndEffector);
                % Visualize parts
                if obj.PartOnRobot~=0
                    obj.Parts{obj.PartOnRobot}.mesh.Pose = obj.CurrentRobotTaskConfig * trvec2tform([0 0 0.04]);
                    obj.Parts{obj.PartOnRobot}.plotHandle.Matrix = obj.Parts{obj.PartOnRobot}.mesh.Pose;
                end
                drawnow;
                pause(0.05);
            end
        end
        
        function visualizePath(obj, positions)
            poses = zeros(size(positions,2),3);
            for i=1:size(positions,2)               
                poseNow = getTransform(obj.Robot, positions(:,i)', obj.RobotEndEffector);
                poses(i,:) = [poseNow(1,4), poseNow(2,4), poseNow(3,4)];
            end
            obj.PathHandle = plot3(poses(:,1), poses(:,2), poses(:,3),'r-','LineWidth',5);            
            drawnow;
        end
        
        % Display current job state
        function displayState(obj, message)
            disp(message);
        end
        
        % Delete function
        function delete(obj)
            delete(obj.FlowChart)
        end
            
    end
  
end

