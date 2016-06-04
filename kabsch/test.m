clear;
clc;

% load data
aruco_coordinates = h5read('aruco_uwb_output.hdf5', '/aruco_coordinates');
aruco = ones(4,length(aruco_coordinates));
aruco(1:3,:) = aruco_coordinates(2:4,:);

uwb_coordinates = h5read('aruco_uwb_output.hdf5', '/uwb_coordinates');
uwb = ones(4,length(uwb_coordinates));
uwb(1:3,:) = uwb_coordinates(2:4,:);

% Create fake uwb data
% transform
%uwb_fake = [1, 0, 0, 5; 0, 1, 0, 10; 0, 0, 1, 1; 0, 0, 0, 1] * aruco;
% scale
%uwb_fake = 8 .* uwb_fake;
% add noise
%uwb_fake(1:3,:) = uwb_fake(1:3,:) + randn(3,length(uwb_fake));


% Calculate mean and std. deviation
%mean_uwb = mean(uwb_fake(1:3,:), 2);
mean_uwb = mean(uwb(1:3,:), 2);
mean_aruco = mean(aruco(1:3,:), 2);
%std_uwb = std(uwb_fake(1:3,:),0,2);
std_uwb = std(uwb(1:3,:),0,2);
std_aruco = std(aruco(1:3,:),0,2);


% calculate scale
scale = mean(std_uwb./std_aruco);
t = [mean_uwb - 8.*mean_aruco];

% normalize data
aruco = scale.*(aruco(1:3,:) - mean_aruco*ones(1,length(aruco)));
uwb = uwb(1:3,:) - mean_uwb*ones(1,length(uwb));
%uwb_fake = uwb_fake(1:3,:) - mean_uwb*ones(1,length(uwb_fake));


[U, r, lrms] = Kabsch(aruco, uwb);
%[U, r, lrms] = Kabsch(aruco, uwb_fake);