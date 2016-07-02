clear;
clc;

TOL = 5; % tolerance in milliseconds

% Load data sets
aruco_coordinates = h5read('aruco_uwb_output.hdf5', '/aruco_coordinates');
uwb_coordinates = h5read('aruco_uwb_output.hdf5', '/uwb_coordinates');

% Matching of the datasets according to their time stamps
pos = 1;
offset = 2;

for i=1:length(uwb_coordinates)
    for j=offset:length(aruco_coordinates)
       % value where aruco time > uwb time 
       if uwb_coordinates(1,i) < aruco_coordinates(1,j)
           % check the one before
           if abs(uwb_coordinates(1,i) - aruco_coordinates(1,j-1)) < TOL
               uwb(:,pos) = uwb_coordinates(:,i);
               aruco(:,pos) = aruco_coordinates(:,j-1);
               pos = pos + 1;
               offset = j;
               continue;
           end
       end
    end
end

max_diff = max(abs(uwb(1,:) - aruco(1,:)))

aruco(1:3,:) = aruco(2:4,:);
index = true(1, size(aruco,1));
index(4) = false;
aruco = aruco(index, :);
uwb(1:3,:) = uwb(2:4,:);
index = true(1, size(uwb,1));
index(4) = false;
uwb = uwb(index, :);

% Exchange x and z axis to fit to aruco coordinates, exchange x and y axis
% too.
% tmp = uwb(1,:);
% tmp2 = uwb(2,:);
% uwb(2,:) = uwb(1,:);
% uwb(3,:) = tmp;
% uwb(1,:) = -tmp;
%  
% aruco(2,:) = -aruco(2,:);

% Modify aruco data to check if rotation and translation is correct
% aruco = (sqrt(2)/2).*[1, -1, 0; 1, 1, 0; 0, 0, 1] * aruco;
% aruco = aruco + repmat(100, [3, size(aruco, 2)]);

% Calculate mean and std. deviation
mean_uwb = mean(uwb(1:3,:), 2);
mean_aruco = mean(aruco(1:3,:), 2);
std_uwb = std(uwb(1:3,:),0,2);
std_aruco = std(aruco(1:3,:),0,2);


% normalize data
%aruco = scale.*(aruco(1:3,:) - mean_aruco*ones(1,length(aruco)));

%aruco(1,:) = scale(1).*(aruco(1,:) - mean_aruco(1)*ones(1,length(aruco(1,:))));
%aruco(2,:) = scale(2).*(aruco(2,:) - mean_aruco(2)*ones(1,length(aruco(1,:))));
%aruco(3,:) = scale(3).*(aruco(3,:) - mean_aruco(3)*ones(1,length(aruco(1,:))));

aruco_centred(1,:) = (aruco(1,:) - mean_aruco(1)*ones(1,length(aruco(1,:))));
aruco_centred(2,:) = (aruco(2,:) - mean_aruco(2)*ones(1,length(aruco(1,:))));
aruco_centred(3,:) = (aruco(3,:) - mean_aruco(3)*ones(1,length(aruco(1,:))));
%uwb = uwb(1:3,:) - mean_uwb*ones(1,length(uwb));
uwb_centred(1,:) = uwb(1,:) - mean_uwb(1)*ones(1,length(uwb(1,:)));
uwb_centred(2,:) = uwb(2,:) - mean_uwb(2)*ones(1,length(uwb(2,:)));
uwb_centred(3,:) = uwb(3,:) - mean_uwb(3)*ones(1,length(uwb(3,:)));

% calculate scale
%scale = (std_uwb./std_aruco);
scale = norm(uwb_centred)/norm(aruco_centred);

% Scale aruco
aruco = scale .* aruco;

% Perform Kabsch
[U, r, lrms] = Kabsch(aruco, uwb);