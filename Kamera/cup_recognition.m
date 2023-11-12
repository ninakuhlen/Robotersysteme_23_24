cam = webcam("Intel(R) RealSense(TM) Depth Camera 435 with RGB Module RGB");
yolov4 = yolov4ObjectDetector("csp-darknet53-coco");

while true
    % Erfassen Sie ein Bild von der Webcam
    img = snapshot(cam);

    % Führen Sie die Objekterkennung durch
    detections = detect(yolov4, img);
    detectionSize = size(detections);
    cupDetections = reshape(detections, detectionSize(1), []);
    
    % Annahme: detections hat mindestens 4 Spalten
    validColumns = size(detections, 2);
    columnsToUse = min(4, validColumns);
    
    % Überprüfen Sie, ob es überhaupt erkannte Objekte gibt und markieren Sie nur Tassen
    %if ~isempty(detections)
        % Filtern basierend auf Klassennamen und Vertrauenswert
     %   cupIdx = strcmp(detections(:, 1), 'cup') & detections(:, 2) > 0.5;  % Annahme: Vertrauenswert > 0.5
      %  cupDetections = detections(cupIdx, :);
    
        % Zeigen Sie das Bild mit den erkannten Tassen an
        label = ["cup"];
        %position = [23 373 60 66; 35 185 77 81; 77 107 59 26];
        img = insertObjectAnnotation(img, "rectangle", cupDetections, label); %, FontColor="black") %cupDetections(:, 1:4) cupDetections(:, 1));
    %end
    
    % Anzeigen des Bilds
    imshow(img);

    % Beenden Sie die Schleife mit der 'ESC'-Taste
    k = waitforbuttonpress;
    if k == 1
        break;
    end
end

% Stoppen Sie die Webcam
clear('cam');