function qi2b = quatFromEulerAngles(angPos)
% Get quat inertial to body for Euler angle sequence roll, pitch, yaw.
% quat is scalar-first


qi2b(1) = cos(angPos(1)/2)*cos(angPos(2)/2)*cos(angPos(3)/2) + ...
    sin(angPos(1)/2)*sin(angPos(2)/2)*sin(angPos(3)/2);

qi2b(2,1) = cos(angPos(1)/2)*cos(angPos(2)/2)*sin(angPos(3)/2) - ...
    sin(angPos(1)/2)*sin(angPos(2)/2)*cos(angPos(3)/2);

qi2b(3,1) = cos(angPos(1)/2)*sin(angPos(2)/2)*cos(angPos(3)/2) + ...
    sin(angPos(1)/2)*cos(angPos(2)/2)*sin(angPos(3)/2);

qi2b(4,1) = sin(angPos(1)/2)*cos(angPos(2)/2)*cos(angPos(3)/2) - ...
    cos(angPos(1)/2)*sin(angPos(2)/2)*sin(angPos(3)/2);



end