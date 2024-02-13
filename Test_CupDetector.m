clear all;

addpath('.\camera');

% load image for calculation of transformation matrix from pcs to rcs
image = imread("C:\Users\Studium\Documents\GitHub\Robotersysteme_23_24\camera\test images\cup.png");

image = rgb2gray(image);
image = imbinarize(image, "global");
image = imcomplement(image);

imshow(image)

[centers, radii, metric] = imfindcircles(image,[50 100],"Sensitivity", 0.90)
centersStrong5 = centers(1,:); 
radiiStrong5 = radii(1);
metricStrong5 = metric(1);

viscircles(centers, radii,'EdgeColor','b');