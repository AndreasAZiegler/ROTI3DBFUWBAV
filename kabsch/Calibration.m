clear;
clc;

TOL = 10; % tolerance in milliseconds

aruco_coordinates = h5read('aruco_output.hdf5', '/aruco_coordinates');
uwb_coordinates = h5read('aruco_output.hdf5', '/uwb_coordinates');

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