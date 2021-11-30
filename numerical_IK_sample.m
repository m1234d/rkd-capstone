function goal_angles = numerical_IK_sample(robot, goal_position, initial_theta)
% This function takes a "Robot" object (containing a end_effector
% function), a column vector goal position (in this case, just x, y, z position)
% and an initial set of joint angles (in column vector form).
%
% It should return a column vector of destination angles.

%% Use lsqnonlin() to actually run the optimization in MATLAB.  First, we set
%% some minimizer options. As with gradient descent, increase the number of
%% iterations for a better solution.
options = optimset( 'algorithm', {'levenberg-marquardt',.1}, ...
                    'DerivativeCheck', 'off', ...
                    'TolX', .002, ...
                    'Display', 'off', ...
                    'MaxIter', 10 );

%% The first task is to generate an "error function", which describes how
%% far from the goal any given set of joint angles is.  In past assignments,
%% we just used the square of the Euclidean distance; this is what we will do
%% below.
function err = my_error_function(xyz_target, robot, theta)
  actual_pos = robot.end_effector(theta);
  %% Just get the position component (modify this for position and orientation
  %% goals!)
  actual_pos = actual_pos(1:3);
  err = (xyz_target - actual_pos).^2;
end

%% Actually run the optimization to generate the angles to get us (close) to
%% the goal.
goal_angles = lsqnonlin( @(theta) my_error_function(...
  goal_position, robot, theta), ...
  initial_theta, [], [], options );

end
