syms theta1 alpha1 d1 a1

TrZAxShift1=[
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d1;
    0, 0, 0, 1]

RotZLink1=[
    cos(theta1), -sin(theta1), 0, 0;
    sin(theta1), cos(theta1), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1]

TrXArmLen1=[
    1, 0, 0, a1;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1]

RotXAxShift1=[
    1, 0, 0, 0;
    0, cos(alpha1), -sin(alpha1), 0;
    0, sin(alpha1), cos(alpha1), 0;
    0, 0, 0, 1]

DH1 = RotZLink1 * TrZAxShift1 * TrXArmLen1 * RotZLink1

% Achse 5
syms theta5 alpha5 d5 a5

TrZAxShift5=[
    1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, 1, d1;
    0, 0, 0, 1]

RotZLink5=[
    cos(theta1), -sin(theta1), 0, 0;
    sin(theta1), cos(theta1), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1]

TrXArmLen5=[
    1, 0, 0, a1;
    0, 1, 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1]

RotXAxShift5=[
    1, 0, 0, 0;
    0, cos(alpha1), -sin(alpha1), 0;
    0, sin(alpha1), cos(alpha1), 0;
    0, 0, 0, 1]

DH1 = RotZLink1 * TrZAxShift1 * TrXArmLen1 * RotZLink1

