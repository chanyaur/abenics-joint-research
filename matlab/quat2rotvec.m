function v = quat2rotvec(q)
%QUAT2ROTVEC  Quaternion [w x y z] -> rotation vector (axis*angle), 3x1.
q = q(:).' / norm(q);
if q(1) < 0, q = -q; end            % shortest rotation
vpart = q(2:4); s = norm(vpart);
if s < 1e-12
    v = 2*vpart(:);                 % small-angle limit
else
    ang = 2*atan2(s, q(1));
    v = (ang/s) * vpart(:);
end
end
