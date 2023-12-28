%% Messen von planaren Objekten mit einer kalibrierten Kamera

%% ___________1___________ Einlesen des Bildes der zu messenden Objekte

% Vergrößerungsfaktor für die Anzeige
magnification = 30;

% Setze den vollständigen Pfad zur Bilddatei zusammen
Bild = fullfile('C:\Users\William\Desktop\handycamera', '20231115183833_3_30cm.jpg');

% Lade das Bild mit dem benutzerdefinierten Pfad
imOrig = imread(Bild);

% Hier kannst du weiter mit deiner Bildverarbeitung arbeiten
figure; imshow(imOrig, 'InitialMagnification', magnification);
title("Eingangsbild");

%% ___________2___________ Bild entzerren

% Da das Objektiv wenig Verzerrung eingeführt hat, verwenden wir die
% 'full'-Ausgabeansicht, um zu illustrieren, dass das Bild entzerrt wurde.
% Wenn die Standardoption 'same' verwendet würde, wäre es schwer, einen
% Unterschied zum Originalbild zu bemerken. Beachte die kleinen schwarzen Ränder.
[img, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
figure; imshow(img, 'InitialMagnification', magnification);  
title("Entzerrtes Bild");

%% ___________3___________ Segmentierung und Erkennung der Gläser


% Konvertiere das Bild in den HSV-Farbraum.
imHSV = rgb2hsv(img);

% Extrahiere den Sättigungskanal.
saturation = imHSV(:, :, 2);

% Starte mit einem Schwellenwert von 0.1
threshold = 0.1;

% Iteriere durch die Schwellenwerte, bis ein Objekt mit Breite von 128 mm erkannt wird.
while true
    % Anwenden des aktuellen Schwellenwerts.
    imGlas = (saturation > threshold);

    % Gefundene verbundene Komponenten extrahieren.
    blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true, ...
        'CentroidOutputPort', false, ...
        'BoundingBoxOutputPort', true, ...
        'MinimumBlobArea', 200, 'ExcludeBorderBlobs', true);
    [areas, boxes] = step(blobAnalysis, imGlas);

    % Verbundene Komponenten nach Fläche absteigend sortieren.
    [~, idx] = sort(areas, 'Descend');

    % Die größten Komponenten extrahieren.
    boxes = double(boxes(idx(1:1), :));

    % Wenn ein Objekt gefunden wurde und die Breite 128 mm beträgt, beende die Schleife.
    if ~isempty(boxes)
        % Breite der BoundingBox als Durchmesser verwenden.
        diameter = boxes(3);  % Beispiel: Annahme, dass die Breite der BoundingBox den Durchmesser repräsentiert.
        if diameter >= 80
            break;  % Die Schleife beenden, wenn ein Objekt mit der gewünschten Breite gefunden wurde.
        end
    end

    % Erhöhe den Schwellenwert für den nächsten Iterationsschritt.
    threshold = threshold + 0.01
end

% Größe des Bildes für die Anzeige reduzieren.
scale = magnification / 100;
imDetectedGlaeser = imresize(img, scale);

% Labels für die Gläser einfügen.
imDetectedGlaeser = insertObjectAnnotation(imDetectedGlaeser, 'rectangle', ...
    scale * boxes, 'Glas');

% Anzeigen der erkannten Gläser.
figure; 
imshow(imDetectedGlaeser);
title('Erkannte Gläser');



%% _______________4______________ Berechnung der Extrinsischen Parameter

% Bildpunkte so anpassen, dass sie im Koordinatensystem des Originalbildes
% vor der Entzerrung ausgedrückt werden. Diese Anpassung macht sie
% kompatibel mit dem für das Originalbild berechneten cameraParameters-Objekt.

% Erkenne das Schachbrettmuster.
[imagePoints, boardSize] = detectCheckerboardPoints(img);

% Füge newOrigin zu jedem Bildpunkt hinzu.
imagePoints = imagePoints + newOrigin;

% Extrahiere Kameraparameter.
camIntrinsics = cameraParams.Intrinsics;

% Berechne die extrinsischen Parameter der Kamera.
camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, camIntrinsics);

%%  _______________5______________ Messen des Durchmessers des ersten Glases
 
% Passe die oberen linken Ecken der BoundingBoxen für die Verschiebung des
% Koordinatensystems durch die Entzerrung mit der Ausgabeansicht 'full' an.
% Dies wäre nicht notwendig, wenn die Ausgabe 'same' wäre. Die Anpassung
% macht die Punkte kompatibel mit den cameraParameters des Originalbildes.
boxes = boxes + [newOrigin, 0, 0];

% Erhalte die obere linke und die obere rechte Ecke.
box1 = double(boxes(1, :));
imagePoints1 = [box1(1:2); ...
                box1(1) + box1(3), box1(2)];

% Erhalte die Weltkoordinaten der Ecken            
worldPoints1 = img2world2d(imagePoints1, camExtrinsics, camIntrinsics);

% Berechne den Durchmesser des Glases in Millimetern.
d = worldPoints1(2, :) - worldPoints1(1, :);
durchmesserInMillimetern = hypot(d(1), d(2));
fprintf("Gemessener Durchmesser des Glases = %0.2f mm\n", durchmesserInMillimetern);

%% _______________6______________ Messen der Entfernung zum ersten Glas

% Berechne das Zentrum des ersten Glases im Bild.
zentrum1_image = box1(1:2) + box1(3:4)/2;

% Konvertiere zu Weltkoordinaten.
zentrum1_world  = img2world2d(zentrum1_image, camExtrinsics, camIntrinsics);

% Denke daran, die z-Koordinate 0 hinzuzufügen.
zentrum1_world = [zentrum1_world 0];

% Berechne die Entfernung zur Kamera.
kameraPose = extr2pose(camExtrinsics);
kameraPosition = kameraPose.Translation
entfernungZurKamera = norm(zentrum1_world - kameraPosition);
fprintf("Entfernung von der Kamera zum Glas = %0.2f mm\n", ...
    entfernungZurKamera);
