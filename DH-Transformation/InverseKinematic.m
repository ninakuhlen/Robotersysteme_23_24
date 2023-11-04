classdef InverseKinematic
    % All equations are derived from the paper:
    % Andersen (2018) Kinematics of a UR5
    % https://blog.aau.dk/wp-signup.php?new=rasmusan
   properties
       % Set 1 - Paper
       alpha = [0; pi/2; 0; 0; pi/2; -pi/2];
       a = [0; 0; -0.24355; -0.2132; 0; 0];
       d = [0.15185; 0; 0; 0.13105; 0.08535; 0.0921];
       theta = zeros(8,6);
       % Set 2
       % alpha = [pi/2; 0; 0; pi/2; -pi/2; 0];
       % a = [0; -0.24355; -0.2132; 0; 0; 0];
       % d = [0.15185; 0; 0; 0.13105; 0.08535; 0.0921];
       % theta = [0; 0; 0; 0; 0; 0];
   end
   methods
       function DH = DHTransform(self, alpha, a, d, theta)
           DH = [cos(theta) -sin(theta) 0 a;
               sin(theta)*cos(alpha) cos(theta)*cos(alpha) -sin(alpha) -sin(alpha)*d;
               sin(theta)*sin(alpha) cos(theta)*sin(alpha) cos(alpha) cos(alpha)*d;
               0 0 0 1];
       end

       function T = Transform(self, from, to, row)
               T = eye(4);
               if from <= to
                   from = from + 1;
                   for i = from:to
                       self.theta(row,i);
                       T = T * inv(self.DHTransform(self.alpha(i), self.a(i), self.d(i), self.theta(row,i)));
                   end
               elseif from > to
                   to = to + 1;
                   for i = from:-1:to
                       T = T * self.DHTransform(self.alpha(i), self.a(i), self.d(i), self.theta(i));
                   end
           end
       end

       function get_thetas(self, x, y, z, roll, pitch, yaw)
           % Attention! The variables use the following naming
           % convention:
           % * T06 is the transformation from frame 0 to frame 6
           % * P06 is the origin of frame 0 seen from frame 6

           % T06 ist eigentlich der Input! Das erzeugt allerdings komplexe
           % Ergebnisse
           T60 = TranslationXYZ(x, y, z);
           T60(1:3, 1:3) = eul2rotm([roll, pitch, yaw]);

           % points derived from T60
           P60 = T60 * [0; 0; 0; 1];
           P50 = T60 * [0; 0; -self.d(6); 1];

           % vectors derived from T60 - Warum keine Transformation von F0
           % nach F6
           X06 = [1; 0; 0; 1];
           Y06 = [0; 1; 0; 1];

           % defining an acos function that returns both solutions
           CustAcos = @(x) [acos(x),-acos(x)];
           
           % Calculating theta1 - Equation 9 - 2 Solutions
           theta1 = atan2(P50(2), P50(1)) ...
               + CustAcos(self.d(4) / norm([P50(1); P50(2)])) ...
               + pi/2
           
           % storing theta1 - Placeholder for selection method
           self.theta(1:2, 1) = theta1(1);
           self.theta(3:4, 1) = theta1(2);
           self.theta(5:6, 1) = theta1(1);
           self.theta(7:8, 1) = theta1(2);

           % Calculating theta5 - Equation 12 - 4 Solutions
           for row = 1:2:(size(self.theta,1))
               theta5 = CustAcos((P60(1)*sin(self.theta(row, 1)) ...
                   - P60(2)*cos(self.theta(row, 1)) ...
                   - self.d(4)) ...
                   ./ self.d(6))
    
               % storing theta5 - Placeholder for selection method
               self.theta(row:row+1, 5) = theta5;
           end

           % Calculating theta6 - Equation 1
           theta6 = atan2(...
               (-X06(2) * sin(self.theta(:, 1)) + Y06(2)*cos(self.theta(:, 1))) ...
               ./sin(self.theta(:, 5)), ...
               (X06(1) * sin(self.theta(:, 1)) - Y06(1)*cos(self.theta(:, 1))) ...
               ./sin(self.theta(:, 5))...
               )

           % storing theta6
           self.theta(:,6) = theta6;

           % Calculating theta3
           for row = 1:(size(self.theta,1)/2)
               T01 = self.Transform(0,1,row);
               T45 = self.Transform(4,5,row);
               T56 = self.Transform(5,6,row);
               T41 = T01 * T60 * T56 * T45;
               P41 = T41(1:3,4);
               
               % Equation 19 - 2 Solutions
               theta3 = CustAcos( ...
                   (power(norm([P41(1); P41(3)]), 2) ...
                   - power(self.a(3), 2) ...
                   - power(self.a(4), 2)) ...
                   / (2 * self.a(3) * self.a(4)) ...
                   )
               
               % storing theta3 - Placeholder for selection method
               self.theta(row, 3) = theta3(1);
               self.theta(row + 4, 3) = theta3(2);

               % Calculating theta4
               for ink = 0:1
                   % Calculating theta2 - Equation 22
                   theta2 = atan2( ...
                       -P41(3), -P41(1) ...
                       ) ...
                       - asin( ...
                       -self.a(4)* sin(self.theta(row + (4 * ink),3)) ...
                       / norm([P41(1); P41(3)]) ...
                       )
                   self.theta(row + (4 * ink), 2) = theta2;

                   T12 = self.Transform(1,2,row + (4 * ink));
                   T23 = self.Transform(2,3,row + (4 * ink));
                   T43 = T23 * T12 * T41;
        
                   X43 = T43 * [1; 0 ; 0 ; 0];
        
                   % Equation 23
                   theta4 = atan2( ...
                       X43(2), X43(1)...
                       )
        
                   % storing theta4
                   self.theta(row + (4 * ink), 4) = theta4;
               end
           end
           self.theta
       end % end function get_thetas
   end % end methods
end % end classdef

