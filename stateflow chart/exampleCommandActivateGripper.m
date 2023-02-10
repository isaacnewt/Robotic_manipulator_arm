function exampleCommandActivateGripper(coordinator, state)
% This class is for internal use and may be removed in a future release
%
%CommandActivateGripper Command function to activate gripper  
%   This command activates and deactivates the gripper. In this implementation, this action has two
%   impacts:
%      - When the gripper is activated, the picked part is added as a collision mesh to the robot rigid body tree so that path planning includes the part in the
%      obstacle avoidance stage. When deactivating the gripper, the placed
%      part is removed from the collision meshes of the rigid body tree.
%      - The visualization is updated to move the object being
%      picked up by the gripper. 
%

% Copyright 2020 The MathWorks, Inc.

        %   Based on the state, decide whether to activate or
        %   deactivate the gripper
       if strcmp(state,'on') == 1
           % Activate gripper 
           coordinator.PartOnRobot = coordinator.NextPart;
           % Add new picked part in collision checking
           partBody = getBody(coordinator.Robot,'pickedPart');
           addCollision(partBody,"sphere", 0.12 , trvec2tform([0 0 0]));
       else
           % Deactivate gripper 
           coordinator.PartOnRobot = 0;
           % Remove dropped part from collision checking
           partBody = getBody(coordinator.Robot,'pickedPart');
           clearCollision(partBody);
       end
       
       % Trigger Stateflow chart Event
       coordinator.FlowChart.nextAction; 
end