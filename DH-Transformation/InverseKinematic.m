classdef InverseKinematic
    % All equations are derived from the paper:
    % Andersen (2018) Kinematics of a UR5
    % https://blog.aau.dk/wp-signup.php?new=rasmusan
   properties
       % Set 1
       alpha = [0; pi/2; 0; 0; pi/2; -pi/2];
       a = [0; 0; -0.24355; -0.2132; 0; 0];
       d = [0.15185; 0; 0; 0.13105; 0.08535; 0.0921];
       theta = [0; 0; 0; 0; 0; 0];
   end
   methods
       function T = Transform(self, from, to)
           T = eye(4);
           if from <= to
               for i = from:to
                   T = T * inv(DHTransform(self.alpha(i), self.a(i), self.d(i), self.theta(i)));
               end
           elseif from > to
               for i = from:-1:to
                   T = T * DHTransform(self.alpha(i), self.a(i), self.d(i), 0);
               end
           end
       end
       function get_thetas(self) % , x, y, z, roll, pitch, yaw)

           % T06 ist eigentlich der Input! Das erzeugt allerdings komplexe
           % Ergebnisse
           % T06 = TranslationXYZ(x, y, z);
           % T06(1:3, 1:3) = eul2rotm([roll, pitch, yaw]);
           % T60 = inv(T06);
           
           % target transform matrix and its inverse
           T06 = self.Transform(1,6);
           T60 = self.Transform(6,1);

           % points derived from T06
           P06 = T06 * [0;0;0;1];
           P05 = T06 * [0; 0; - self.d(6); 1];

           % vectors derived from T06
           X60 = T60 * [1; 0; 0; 1];
           Y60 = T60 * [0; 1; 0; 1];
           

           % Calculating theta1 - Equation 9
           theta1 = atan2(P05(2), P05(1)) ...
               + acos(self.d(4) / norm([P05(1); P05(2)])) ...
               + pi/2

           % Calculating theta5 - Equation 12
           theta5 = acos((P06(1)*sin(theta1) ...
               - P06(2)*cos(theta1) ...
               - self.d(4)) ...
               / self.d(6))

           % Calculating theta6 - Equation 16
           theta6 = atan2(...
               (-X60(2) * sin(theta1) + Y60(2)*cos(theta1)) ...
               /sin(theta5), ...
               (X60(1) * sin(theta1) - Y60(1)*cos(theta1)) ...
               /sin(theta5)...
               )
           
           % Calculating theta3
           T14 = self.Transform(2,4);
           P14 = T14 * [0;0;0;1];
           
           % Equation 19
           theta3 = acos( ...
               power(norm([P14(1); P14(2)]), 2) ...
               - power(self.a(3), 2) ...
               - power(self.a(4), 2) ...
               / (2 * self.a(3) * self.a(4)) ...
               )

           % Calculating theta2 - Equation 22
           theta2 = atan2( ...
               -P14(3), -P14(1) ...
               ) ...
               - asin( ...
               -self.a(4)* sin(theta3) ...
               / norm([P14(1); P14(3)]) ...
               )

           % Calculating theta4
           T34 = self.Transform(4,4);
           X34 = T34 * [1; 0 ; 0 ; 1];
           
           % Equation 23
           theta4 = atan2( ...
               X34(2), X34(1)...
               )
       end
   end
end

