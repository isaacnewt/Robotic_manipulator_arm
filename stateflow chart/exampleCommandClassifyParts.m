function exampleCommandClassifyParts(coordinator)
%
%CommandClassifyParts Classify the parts to determine where to place them
%   This command classifies the detected parts using a numeric type: type 1
%   or type 2.
%
% Copyright 2020 The MathWorks, Inc.

    % In this method, the classification is assumed to be known. In
    % practice, this may be replaced by more complex classification
    % tools. For example, the robot can use camera images or point
    % clouds  from 3D scanners to classify objects in the scene.
    if ~isempty(coordinator.DetectedParts)
        coordinator.DetectedParts{1}.type = 1;
        coordinator.DetectedParts{2}.type = 1;
        coordinator.DetectedParts{3}.type = 2;
    end

   % Trigger Stateflow chart Event
   coordinator.FlowChart.partsClassified;       
end