function eul = quat2eul_xyz(q)
%QUAT2EUL_XYZ  Quaternion [w x y z] -> intrinsic XYZ Euler angles [r p y].
%   Inverse of EUL2QUAT_XYZ for R = Rx(r)*Ry(p)*Rz(y). Replacement for
%   quat2eul(q,'XYZ') without the Robotics System Toolbox.
R = quat2rotm_wxyz(q);
sp = max(min(R(1,3), 1), -1);      % clamp for numerical safety
pp = asin(sp);
r  = atan2(-R(2,3), R(3,3));
yy = atan2(-R(1,2), R(1,1));
eul = [r, pp, yy];
end
