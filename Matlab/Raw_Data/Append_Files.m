%% Alignment of images towards centre of image. 

clc;	% Clear command window.
clear;	% Delete all variables.
close all;	% Close all figure windows except those created by imtool.


%% HARDCODED INPUTS
filePattern_BIN = fullfile('*.BIN');
FileName = 'ROVCOMP.BIN';

%% Load Binary Data in Loop
Files_BIN = dir(filePattern_BIN);
for k = 1:length(Files_BIN)
    Filename_BIN = fullfile(['ROV' num2str(k) '.BIN']);
end
disp(k);
s = sprintf('ROV%d.BIN+', 1:(k-1));
argument = ['copy ', s, ['ROV', num2str(k), '.BIN'], ' ', FileName];
disp(argument);
system(argument);
disp('Job terminated!')

