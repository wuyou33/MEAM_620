% Add addicitonal initialization if you want to.
% You can use this space to pre-compute the trajectory to avoid repeatly compute the same trajectory in every call of the "trajectory_generator" function

% Generate trajectory
disp('Generating Trajectory ...');
trajectory_generator([], map, path);
