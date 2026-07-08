function q = eul2quat_xyz(eul)
%EUL2QUAT_XYZ  Intrinsic XYZ Euler angles -> quaternion [w x y z].
%   q = EUL2QUAT_XYZ([r p y]) builds R = Rx(r)*Ry(p)*Rz(y) as a quaternion,
%   exactly matching the sequence used in the onlyJoints.slx abenics_fk block.
%   No Robotics System Toolbox dependency.
r = eul(1); pp = eul(2); yy = eul(3);
qx = [cos(r/2)  sin(r/2)  0         0        ];
qy = [cos(pp/2) 0         sin(pp/2) 0        ];
qz = [cos(yy/2) 0         0         sin(yy/2)];
q = quatmul(quatmul(qx, qy), qz);
end
