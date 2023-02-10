function exampleCommandDetectParts(coordinator)
%
%CommandDetectParts Detect parts and identify their poses
%   In this simplified workflow, the parts are detected using a list in the
%   coordinator that both indicates the part exists and provides its
%   starting pose.


% Copyright 2020 The MathWorks, Inc.

% This function detects parts using a list in the coordinator that also provides the pose. 
% This is a simplified workflow that could be replaced using sensor measurements
% such as camera images or point clouds form 3D scanners.
            
        if ~isempty(coordinator.Parts) && coordinator.NextPart<=length(coordinator.Parts)
            coordinator.DetectedParts = coordinator.Parts;
            % Trigger event 'partDetected' on Stateflow
            coordinator.FlowChart.partsDetected;
            return;
        end
        coordinator.NumDetectionRuns = coordinator.NumDetectionRuns +1;

        % Trigger Stateflow chart Event
        coordinator.FlowChart.noPartsDetected; 
   
end