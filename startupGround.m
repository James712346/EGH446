%% close previously open model
close_system('sl_groundvehicleDynamics',0);

%% add toolboxes to path
homedir = pwd; 
addpath( genpath(strcat(homedir,[filesep,'toolboxes'])));

cd('toolboxes/MRTB');
startMobileRoboticsSimulationToolbox;

cd(homedir);

%% open current model
open_system('sl_groundvehicleDynamics'); %differential robot

cd(homedir);
