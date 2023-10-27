classdef InverseKinematic
   properties
       a = [0; -0.24355; -0.2132; 0; 0; 0];
       alpha = [pi/2; 0; 0; pi/2; -pi/2; 0];
       d = [0.15185; 0; 0; 0.13105; 0.08535; 0.0921];
   end
   methods
       function T = Transform(self, from, to)
           T = eye(4);
           if from < to
               for i = from:to
                   T = T * DHTransform(self.alpha(i), self.a(i), self.d(i), 0);
               end
           elseif from > to
               for i = from:-1:to
                   T = T * DHTransform(self.alpha(i), self.a(i), self.d(i), 0);
               end
           end
       end
       function theta1 = get_theta1(self)
           T05 = self.Transform(1,5);
           v = [0; 0; - self.d(6); 1];

           % Equation 5
           P05 = T05 * v;
           
           % Equation 7
           phi1 = atan2(P05(2), P05(1));

           % Equation 8 ACHTUNG ! positiv oder negativ
           %huhu = self.d(4) / abs([P05(1); P05(2)]);
           phi2 = acos(self.d(4) / abs([P05(1); P05(2)]));
           
           % Equation 6
           theta1 = phi1 + (phi2 + pi/2);
       end
       function  theta5 = get_theta5(self, P06)
           theta1 = self.get_theta1();

           % Equation 12
           huhu = (P06(1)*sin(theta1) - P06(2)*cos(theta1)- self.d(4)) / self.d(6)
           theta5 = acos((P06(1)*sin(theta1)- P06(2)*cos(theta1)- self.d(4)) / self.d(6));
       end
       function theta6 = get_theta6(self, P06)
           theta1 = self.get_theta1();
           theta5 = self.get_theta5(P06);
           Y0 = [0; 1; 0; 1];
           X0 = [1; 0; 0; 1];
           T60 = self.Transform(6,1);
           Y60 = T60 * Y0;
           X60 = T60 * X0;
           
           % Equation 16
           theta6 = atan2((-X60(2) * sin(theta1) + Y60(2)*cos(theta1))/sin(theta5), ...
               (X60(1) * sin(theta1) - Y60(1)*cos(theta1))/sin(theta5));
       end






   end
end

