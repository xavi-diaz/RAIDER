function moveVertical(left_cmd, right_cmd)

% uses setTargetPosition to specify positions about the “hip_swing”, 
% “upper_knee” and “ankle_swing” joints of either the left or right legs 
% or both to achieve a vertical movement of the leg/s

global targetPosition;

setTargetPosition(targetPosition(1),targetPosition(2),targetPosition(3),targetPosition(4),targetPosition(5),targetPosition(6),targetPosition(7),targetPosition(8),targetPosition(9),targetPosition(10),targetPosition(11),targetPosition(12),targetPosition(13)+right_cmd, targetPosition(14)+left_cmd,targetPosition(15)-1.75*right_cmd,targetPosition(16)-1.75*left_cmd, targetPosition(17)+0.75*right_cmd, targetPosition(18)+0.75*left_cmd,targetPosition(19),targetPosition(20)); 


end

