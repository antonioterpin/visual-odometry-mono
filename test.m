close all; 
clear all;
clc;
path(pathdef); % Reset paths
addpath(genpath('src')); % Source code
%% Configuration & pipeline blocks loading
% Change the following line to load a different configuration.
% Alternatively, you can specify each block in the configuration object.
configFile = 'config.json';
configuration = Config.loadFromFile(configFile); 
inputHandler = configuration.InputHandler;
detector = configuration.DetectorHandler;

%%

index = 90;
image1 = inputHandler.getImage(index);
image2 = inputHandler.getImage(index + 1);
[keypoints1, descriptors1] = detector.extractFeatures(image1);
[keypoints2, descriptors2] = detector.extractFeatures(image2);
corners1 = detectHarrisFeatures(image1);
corners2 = detectHarrisFeatures(image2);
[features1,valid_points1] = extractFeatures(image1,corners1);
[features2,valid_points2] = extractFeatures(image2,corners2);
indexPairs = matchFeatures(features1,features2)
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);

[~, p1, p2] = detector.getMatches(descriptors1,descriptors2,keypoints1,keypoints2);



subplot(2,1,1);
% showMatchedFeatures(image1,image2,p1,p2);
subplot(2,1,2);
showMatchedFeatures(image1,image2,matchedPoints1,matchedPoints2);
