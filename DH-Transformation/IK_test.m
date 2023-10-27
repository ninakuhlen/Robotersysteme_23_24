addpath('/home/lbeissner/RSY/');



obj = InverseKinematic();

T = obj.Transform(1, 6);

theta1 = obj.get_theta1();
theta5 = obj.get_theta5([0.3;0.3;0.3])
%theta6 = obj.get_theta6([10;10;10])