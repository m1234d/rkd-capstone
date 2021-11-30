function [ trajectory ] = linear_workspace_trajectory(robot, start_theta, goal_pos, num_points)
% linear_workspace_trajectory
%
%   Returns a matrix of joint angles, where each column represents a single
%   timestamp. This matrix is length(start_theta) rows and num_points columns.
%
%   Using these joint angles, the end effector linearly interpolates from the
%   location given configuration start_theta to goal_pos, in the workspace.
%
%   'robot' is the Robot object to use for any required IK calculations.
%
%   'start_theta' is a column vector of joint angles
%
%   'goal_pos' is a 2 or 3 element column vector representing the goal position
%   (or position and orientation) of the end effector.
%
%   'num_points' is the number of columns in the resulting trajectory.

% --------------- BEGIN STUDENT SECTION ----------------------------------

trajectory = zeros(size(start_theta,1), num_points);
positions = zeros(size(goal_pos,1), num_points);

% We know the first column:
trajectory(:, 1) = start_theta;

% HINT: it may be useful to first calculate the desired workspace trajectory to
% reference in the loop below
initial_pos = robot.end_effector(start_theta);

if (size(goal_pos, 1) == 2)
    positions(1, :) = linspace(initial_pos(1), goal_pos(1), num_points);
    positions(2, :) = linspace(initial_pos(2), goal_pos(2), num_points);
else
    positions(1, :) = linspace(initial_pos(1), goal_pos(1), num_points);
    positions(2, :) = linspace(initial_pos(2), goal_pos(2), num_points);
    positions(3, :) = linspace(initial_pos(3), goal_pos(3), num_points);
end
% Find the rests:
for col = 2:num_points

    %% Fill in trajectory(:,col) here. HINT: use trajectory(:,col-1) to help!
    trajectory(:, col) = robot.inverse_kinematics(trajectory(:, col-1), positions(:, col));
% --------------- END STUDENT SECTION ------------------------------------
end

end
