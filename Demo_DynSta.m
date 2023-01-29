%% 3D Voxel Grid
% Inputs: Velodyne points and GPS/IMU localization
% Output: Static / Dynamic environment modeling
% Alireza Asvadi, 2015 July
%% clear memory & command window
clc
clear all
close all
time = [];
for i = 1:1
    if i == 2 
        continue
    end

%% setting 
st           = Fstt(i);
%% main
for frame    =  st.st.st : st.st.tn;             % frame number 1: 25
tic
%% dynamic / static modeling    
[In, prm]    = Fint(st, frame);                  % ground parameters and voxelize integrate points 
Bm           = Fmdl(In.mat, prm, st, frame);     % remove dynamic voxels and build the background model
Fm           = Ffrg(Bm.mat, prm, st, frame);     % compute foreground voxels
%% discriminative analysis
[Bg, ~, ~]   = Fltr(Bm, Fm, st, 100);            % background model
[Fg, ~, ~]   = Fltr(Fm, Bm, st, 5);              % foreground model
toc
disp(['运行时间: ',num2str(toc)]);
time = [time;toc];
%% points extract
% Fg_points = Fptext(Fg,prm, st, frame);
% label = Flabel(Fg_points.ind, st, frame);
%% plot
% Fplot(st, Bg, Fg, prm, frame)
% Fplot_fst(st, Bg, Fg, prm, frame)
end
dlmwrite('/media/yihang/LYH/kitti_tracking/data_tracking_dataset/training/predict_points_16/time.txt',time);
end

