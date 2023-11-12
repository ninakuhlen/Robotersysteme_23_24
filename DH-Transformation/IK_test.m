obj = InverseKinematic();

obj.get_thetas(-0.222,0.222,0.222, 0, 0.5, 0);
obj.getCurrentThetaDeg()

% obj.get_thetas(0.08,0.113,0.805, 0, 0.5, 0);
% obj
% 
% obj.get_thetas(0.08,0.113,2.805, 0, 0.5, 0);
% obj


gd = TranslationXYZ(-0.222,0.222,0.222);
gd(1:3, 1:3) = eul2rotm([0, 0.5, 0]);

ur5inv(gd) * 180 / pi