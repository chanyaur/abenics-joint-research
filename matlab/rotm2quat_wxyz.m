function q = rotm2quat_wxyz(R)
%ROTM2QUAT_WXYZ  3x3 rotation matrix -> quaternion [w x y z] (no toolbox).
tr = trace(R);
if tr > 0
    S = sqrt(tr+1)*2;
    q = [0.25*S, (R(3,2)-R(2,3))/S, (R(1,3)-R(3,1))/S, (R(2,1)-R(1,2))/S];
elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    S = sqrt(1+R(1,1)-R(2,2)-R(3,3))*2;
    q = [(R(3,2)-R(2,3))/S, 0.25*S, (R(1,2)+R(2,1))/S, (R(1,3)+R(3,1))/S];
elseif R(2,2) > R(3,3)
    S = sqrt(1+R(2,2)-R(1,1)-R(3,3))*2;
    q = [(R(1,3)-R(3,1))/S, (R(1,2)+R(2,1))/S, 0.25*S, (R(2,3)+R(3,2))/S];
else
    S = sqrt(1+R(3,3)-R(1,1)-R(2,2))*2;
    q = [(R(2,1)-R(1,2))/S, (R(1,3)+R(3,1))/S, (R(2,3)+R(3,2))/S, 0.25*S];
end
q = q/norm(q);
end
