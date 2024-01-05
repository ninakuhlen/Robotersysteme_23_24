%% Wie geht das?
% Der UR3 gibt auf der IP unter dem port 30003 durchgehend 1060
% verschiedene Werte aus, was der Roboter gerade macht.
% Interessant sind in diesem Fall die TARGET JOINT VELOCITIES;
% solange diese nicht 0 sind, will der UR3 sich noch bewegen.
% Die while Schleife greift die Daten ab und vergleicht sie mit dem 0
% array Stillstandswerte.
function checkDataPos()
    IPP = tcpclient('192.168.1.1',30003);
    data = read(IPP,1060,"int8");
    checkvalue = data(56:61);
    disp(checkvalue)

%     checkvalue = data;
%     while ~isequal(Stillstandswerte,checkvalue)
%         %IPP = tcpclient('192.168.0.9',30003);
%         IPP = tcpclient('192.168.1.1',30003);
%         data = read(IPP,500,"int8");
%         checkvalue = data(61:108);
%         pause(0.05);
%         % debug = 'Ich warte.';     % debug tool kann anzeigen ob die Schleife
%                                     % durchläuft
%         % disp(debug)
%     end
end 