

function moveShoulderSwing(left_cmd, right_cmd)

% uses setTargetPosition to specify positions about the “shoulder_swing” joints of either 
% the left or right shoulders or both to achieve a forward/backwards movement of the shoulders

global targetPosition;

setTargetPosition(targetPosition(1),targetPosition(2),targetPosition(3)+right_cmd,targetPosition(4)+left_cmd,targetPosition(5),targetPosition(6),targetPosition(7),targetPosition(8),targetPosition(9),targetPosition(10),targetPosition(11),targetPosition(12),targetPosition(13), targetPosition(14),targetPosition(15),targetPosition(16),targetPosition(17),targetPosition(18),targetPosition(19),targetPosition(20)); 


end