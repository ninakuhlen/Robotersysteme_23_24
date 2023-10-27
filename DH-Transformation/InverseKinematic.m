classdef InverseKinematic
   properties
       % Set 1
       alpha = [0; pi/2; 0; 0; pi/2; -pi/2];
       a = [0; 0; -0.24355; -0.2132; 0; 0];
       d = [0.15185; 0; 0; 0.13105; 0.08535; 0.0921];
       % Set 2 FALSCH
       %a = [0; -0.24355; -0.2132; 0; 0; 0];
       %alpha = [pi/2; 0; 0; pi/2; -pi/2; 0];
       %d = [0.15185; 0; 0; 0.13105; 0.08535; 0.0921];
       theta = [0; 0; 0; 0; 0; 0];
       % T06 = eye(4);
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
       function theta1 = get_theta1(self)
           % T06 ist eigentlich der Input!
           T06 = self.Transform(1,6);

           % Equation 5
           P05 = self.T06 * [0; 0; - self.d(6); 1];
           
           % Equation 7
           phi1 = atan2(P05(2), P05(1));

           % Equation 8 ACHTUNG ! positiv oder negativ
           phi2 = acos(self.d(4) / norm([P05(1); P05(2)]));
           
           % Equation 6
           theta1 = phi1 + phi2 + pi/2;
           %self.theta(1) = theta1;

       end
       function theta5 = get_theta5(self)
           % T06 ist eigentlich der Input!
           T06 = Transform(1,6);
           P06 = T06 * [0;0;0;1];
           theta1 = self.get_theta1();

           % Equation 12
           theta5 = acos((P06(1)*sin(theta1) - P06(2)*cos(theta1)- self.d(4)) / self.d(6));
           %self.theta(5) = theta5;
       end
       function theta6 = get_theta6(self)
           theta1 = self.get_theta1();
           theta5 = self.get_theta5();
           X0 = [1; 0; 0; 1];
           Y0 = [0; 1; 0; 1];
           
           T60 = self.Transform(6,1);
           Y60 = T60 * Y0;
           X60 = T60 * X0;
           
           % Equation 16
           theta6 = atan2((-X60(2) * sin(theta1) + Y60(2)*cos(theta1))/sin(theta5), ...
               (X60(1) * sin(theta1) - Y60(1)*cos(theta1))/sin(theta5));
           %self.theta(6) = theta6;
       end
       function theta3 = get_theta3(self)
           T14 = self.Transform(2,4);
           P14 = T14 * [0;0;0;1];
           
           % Equation 19
           theta3 = acos(power(norm([P14(1); P14(2)]), 2) - power(self.a(3), 2) ...
               - power(self.a(4), 2) / (2 * self.a(3) * self.a(4)));
           %self.theta(3) = theta3;
       end
       function theta2 = get_theta2(self)
           T14 = self.Transform(2,4);
           P14 = T14 * [0;0;0;1]
           theta3 = self.get_theta3();

           % Equation 22
           theta2 = atan2(-P14(3), -P14(1)) - asin(-self.a(4)* sin(theta3)/ norm([P14(1); P14(3)]));
           %self.theta(2) = theta2;
       end
       function theta4 = get_theta4(self)
           T34 = self.Transform(4,4);
           X34 = T34 * [1; 0 ; 0 ; 1];
           
           % Equation 23
           theta4 = atan2(X34(2), X34(1));
           %self.theta(4) = theta4;
       end






   end
end

