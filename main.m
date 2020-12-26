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
pipeline = MonoVOPipeline(configuration);

%% Ok, that's it with talking. Let's run the pipeline :)
disp('Enjoy the run.');
pipeline.run();