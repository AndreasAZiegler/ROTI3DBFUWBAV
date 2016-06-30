clear;
clc;

TOL = 5; % tolerance in milliseconds

aruco_coordinates = h5read('aruco_vicon_output.hdf5', '/aruco_coordinates');
vicon_coordinates = h5read('aruco_vicon_output.hdf5', '/vicon_coordinates');

pos = 1;
offset = 2;

for i=1:length(vicon_coordinates)
    for j=offset:length(aruco_coordinates)
       % value where aruco time > uwb time 
       if vicon_coordinates(1,i) < aruco_coordinates(1,j)
           % check the one before
           if abs(vicon_coordinates(1,i) - aruco_coordinates(1,j-1)) < TOL
               vicon(:,pos) = vicon_coordinates(:,i);
               aruco(:,pos) = aruco_coordinates(:,j-1);
               pos = pos + 1;
               offset = j;
               continue;
           end
       end
    end
end

max_diff = max(abs(vicon(1,:) - aruco(1,:)))

aruco(1:3,:) = aruco(2:4,:);
index = true(1, size(aruco,1));
index(4) = false;
aruco = aruco(index, :);
vicon(1:3,:) = vicon(2:4,:);
index = true(1, size(vicon,1));
index(4) = false;
vicon = vicon(index, :);

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
mean_vicon = mean(vicon(1:3,:), 2);
mean_aruco = mean(aruco(1:3,:), 2);
std_vicon = std(vicon(1:3,:),0,2);
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
vicon_centred(1,:) = vicon(1,:) - mean_vicon(1)*ones(1,length(vicon(1,:)));
vicon_centred(2,:) = vicon(2,:) - mean_vicon(2)*ones(1,length(vicon(2,:)));
vicon_centred(3,:) = vicon(3,:) - mean_vicon(3)*ones(1,length(vicon(3,:)));

% calculate scale
%scale = (std_uwb./std_aruco);
scale = norm(vicon_centred)/norm(aruco_centred);

aruco = scale .* aruco;


[U, r, lrms] = Kabsch(aruco, vicon);