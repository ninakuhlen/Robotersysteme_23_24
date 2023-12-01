global socketCon;

Robot_IP = '192.168.1.1'; % UR3 IP
Port_NR = 30003; % Communication port

socketCon = tcpip(Robot_IP,Port_NR);
fclose(socketCon); % close prior socket connections

pause(2);
fprintf(1, 'Establishing connection to UR3...\n');
try
    fopen(socketCon);
catch
    error('Connection to UR3 failed.');
    return;
end
fprintf(1, 'Connected to UR3.\n');
pause(2);

%% Setup frequently used variables 

% TODO: Homepos anstelle von Kerze einfügen
homePos = deg2rad([0.0, -90.0, 0.0, -90.0, -90, 0.0]); % Kerze

% Acceleration, velocity, time and blend radius for robot movement
a = 0.3;
v = 0.5;
t = 0;
r = 0.001;

disp('Setup successful.')