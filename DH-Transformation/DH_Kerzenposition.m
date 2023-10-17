%Achse 1

%syms theta1 alpha1 d1 a1
theta1 = 0;
alpha1 = pi/2;
d1 = 0.15185;
a1 = 0;

TrZAxShift1=[
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d1;
    0, 0, 0, 1];

RotZLink1=[
    cos(theta1), -sin(theta1), 0, 0;
    sin(theta1), cos(theta1), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

TrXArmLen1=[
    1, 0, 0, a1;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

RotXAxShift1=[
    1, 0, 0, 0;
    0, cos(alpha1), -sin(alpha1), 0;
    0, sin(alpha1), cos(alpha1), 0;
    0, 0, 0, 1];

DH1 = RotZLink1 * TrZAxShift1 * TrXArmLen1 * RotZLink1


% Achse 2
%syms theta2 alpha2 d2 a2
theta2 = 0;
alpha2 = 0;
d2 = 0;
a2 = -0.24355;

TrZAxShift2=[
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d2;
    0, 0, 0, 1];

RotZLink2=[
    cos(theta2), -sin(theta2), 0, 0;
    sin(theta2), cos(theta2), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

TrXArmLen2=[
    1, 0, 0, a2;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

RotXAxShift2=[
    1, 0, 0, 0;
    0, cos(alpha2), -sin(alpha2), 0;
    0, sin(alpha2), cos(alpha2), 0;
    0, 0, 0, 1];

DH2 = RotZLink2 * TrZAxShift2 * TrXArmLen2 * RotZLink2

% Achse 3
% syms theta3 alpha3 d3 a3
theta3 = 0;
alpha3 = 0;
d3 = 0;
a3 = -0.2132;


TrZAxShift3=[
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d3;
    0, 0, 0, 1];

RotZLink3=[
    cos(theta3), -sin(theta3), 0, 0;
    sin(theta3), cos(theta3), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

TrXArmLen3=[
    1, 0, 0, a3;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

RotXAxShift3=[
    1, 0, 0, 0;
    0, cos(alpha3), -sin(alpha3), 0;
    0, sin(alpha3), cos(alpha3), 0;
    0, 0, 0, 1];

DH3 = RotZLink3 * TrZAxShift3 * TrXArmLen3 * RotZLink3


% Achse 4
%syms theta4 alpha4 d4 a4
theta4 = 0;
alpha4 = pi/2;
d4 = 0.13105;
a4 = 0;

TrZAxShift4=[
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d4;
    0, 0, 0, 1];

RotZLink4=[
    cos(theta4), -sin(theta4), 0, 0;
    sin(theta4), cos(theta4), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

TrXArmLen4=[
    1, 0, 0, a4;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

RotXAxShift4=[
    1, 0, 0, 0;
    0, cos(alpha4), -sin(alpha4), 0;
    0, sin(alpha4), cos(alpha4), 0;
    0, 0, 0, 1];

DH4 = RotZLink4 * TrZAxShift4 * TrXArmLen4 * RotZLink4

% Achse 5
%syms theta5 alpha5 d5 a5
theta5 = 0;
alpha5 = -pi/2;
d5 = 0.08535;
a5 = 0;


TrZAxShift5=[
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d5;
    0, 0, 0, 1];

RotZLink5=[
    cos(theta5), -sin(theta5), 0, 0;
    sin(theta5), cos(theta5), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

TrXArmLen5=[
    1, 0, 0, a5;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

RotXAxShift5=[
    1, 0, 0, 0;
    0, cos(alpha5), -sin(alpha5), 0;
    0, sin(alpha5), cos(alpha5), 0;
    0, 0, 0, 1];

DH5 = RotZLink5 * TrZAxShift5 * TrXArmLen5 * RotZLink5

% Achse 6
%syms theta6 alpha6 d6 a6
theta6 = 0;
alpha6 = 0;
d6 = 0.0921;
a6 = 0;


TrZAxShift6=[
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d6;
    0, 0, 0, 1];

RotZLink6=[
    cos(theta6), -sin(theta6), 0, 0;
    sin(theta6), cos(theta6), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

TrXArmLen6=[
    1, 0, 0, a6;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];

RotXAxShift6=[
    1, 0, 0, 0;
    0, cos(alpha6), -sin(alpha6), 0;
    0, sin(alpha6), cos(alpha6), 0;
    0, 0, 0, 1];

DH6 = RotZLink6 * TrZAxShift6 * TrXArmLen6 * RotZLink6

DHall= DH1 * DH2 * DH3 * DH4 * DH5 * DH6

