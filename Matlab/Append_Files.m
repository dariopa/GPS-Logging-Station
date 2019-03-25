%% Alignment of images towards centre of image. 

clc;	% Clear command window.
clear;	% Delete all variables.
close all;	% Close all figure windows except those created by imtool.


%% HARDCODED INPUTS
LoadPath_BIN = 'Raw_Data/';
filePattern_BIN = fullfile(LoadPath_BIN, '*.bin');
FileName = 'ROVCOMP.BIN';

%% Define MyPath to our local Raw Data
% Check path for ground truth data
if ~isdir(LoadPath_BIN)
	errorMessage = sprintf('Error: The following folder does not exist:\n%s', LoadPath_BIN);
	uiwait(warndlg(errorMessage));
	return;
end

%% Load Binary Data in Loop
Files_BIN = dir(filePattern_BIN);
for k = 1:length(Files_BIN)
    Filename_BIN = fullfile(LoadPath_BIN, ['ROV' num2str(k) '.BIN']);
    disp(['Loading now: ', Filename_BIN]);
    system(['copy ' Filename_BIN ' ' FileName]);
      
end

disp('Job terminated!')

