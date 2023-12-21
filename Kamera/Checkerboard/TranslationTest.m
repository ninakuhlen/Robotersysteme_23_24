% Eingemessene Punkte von Schachbrett in Koordinaten von Basis Roboter
% Werte für x- und y-Translation
P1 = [353,4, -85.62];
P2 = [357.2,  65.4]; % gleichzeitig Ursprung (0,0) Koordinatensystem von Schachbrett aus

% Vektor zwischen P1 und P2 zur Berechnung des Winkels zur x-Achse 
vectorP1P2 = (P2 - P1);

% Winkel zwischen Vektor P1P2 zur Berechnung der Rotation
% Abzug von pi/2, da Vektor P1P2 kolinear zu y-Achse des
% Schachbrettkoordinatensystems liegt, der Winkel theta aber zwischen den
% x-Achsen beider Koordinatensysteme liegen soll
theta = atan(vectorP1P2(2)/vectorP1P2(1)) - (pi/2);

% Transformationsmatrix mit Rotation in xy-Ebene und keiner Rotation in z
% wird für Transformtion von Schachbrett zu Basis Roboter verwendet
% translationMatrix = [rot trans;
%                       0    1   ]
translationMatrix = [cos(theta), -sin(theta), 0,   P2(1);
                     sin(theta), cos(theta),  0,   P2(2);
                     0,          0,           1,   0;
                     0,          0,           0,   1];
xSchachbrett = 0;
ySchachbrett = 10;
zSchachbrett = 0;


testPSchach = [xSchachbrett; -ySchachbrett; zSchachbrett; 1];

testPBasis = translationMatrix * testPSchach