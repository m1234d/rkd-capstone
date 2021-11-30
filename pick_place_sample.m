function [] = pick_place_sample()
% pick_place_sample
%
% pick and place example code; picks up an object at position "A" on
% the table and moves it to position "B".

%% First, we define a couple of helper functions.  You can break these out into
%% separate files if you wish.

% Define a simple function to actually send a series of points to the
% robot, where the 'trajectory' is a matrix of columns of joint angle
% commands to be sent to 'robot' at approximately 'frequency'.
% Note this also commands velocities, although you can choose to only command
% positions if desired, or to add torques to help compensate for gravity.
function [] = command_trajectory(robot, trajectory, frequency)
  %% Setup reusable structures to reduce memory use in loop
  cmd = CommandStruct();

  % Compute the velocity numerically
  trajectory_vel = diff(trajectory, 1, 2);

  % Command the trajectory
  for i = 1:(size(trajectory, 2) - 1)
    % Send command to the robot (the transposes on the trajectory
    % points turns column into row vector for commands).
    cmd.position = trajectory(:,i)';
    cmd.velocity = trajectory_vel(:,i)' * frequency;
    robot.set(cmd);

    % Wait a little bit to send at ~100Hz.
    pause(1 / frequency);
  end

  % Send the last point, with a goal of zero velocity.
  cmd.position = trajectory(:,end)';
  cmd.velocity = zeros(1, size(trajectory, 1));
  robot.set(cmd);
end

% Convenience function to use to hide the internal logic of starting the suction
function [] = pick(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 1;
  suction_cup.set(suction_cmd);
end

% Convenience function to use to hide the internal logic of stopping the suction
function [] = place(suction_cup)
  suction_cmd = IoCommandStruct();
  suction_cmd.e2 = 0;
  suction_cup.set(suction_cmd);
end

% Clear out old information to reduce problems with stale modules
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);

% Connect to physical robot
robot = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});
% Note -- this is how long particular commands that you send to the robot "last"
% before the robot goes limp. Here, we ensure they last for 1 second.
robot.setCommandLifetime(1);
% Load saved control gains, and set these on the robot. These can be tuned to
% improve accuracy, but you should be very careful when doing so.
gains = load('jenga_gains.mat');
robot.set('gains', gains.jenga_gains);

%% Connect to gripper, and initialize some settings properly
gripper = HebiLookup.newGroupFromNames('16384','gripper');
gripper.setCommandLifetime(0);
gripper.setFeedbackFrequency(100);

warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');

%% Get initial position
fbk = robot.getNextFeedback();
initial_thetas = fbk.position'; % (The transpose turns the feedback into a column vector)

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot.startLog('file', fullfile(currentDir, 'robot_data'));
%% command frequency, in Hz
frequency = 100;

% We define our "position 1", "position 2", and a "midpoint" waypoints
% here. We found these points by calling robot.getNextFeedback() from the MATLAB
% command line, and tweaking the results as necessary.
position_1 = [-0.6 0.86 1.4 2.05 0]';
position_2 = [0.32 0.70 1 2.41 0]';

% We keep the last joint equal to the first to ensure the block does not rotate
% as we move. Note this joint points in the opposite direction as the base. For
% position 2, we want to rotate 1/4 turn, so we add pi/2.
position_1(5) = position_1(1);
position_2(5) = position_2(1) + pi/2;

% Add a midpoint for the trajectory, so the robot does not just drag the piece
% across the table.
midpoint = [-0.1906 1.402 1.584 0.4989 0]';
midpoint(5) = position_1(5)*0.5 + position_2(5)*0.5;

% Create a set of "approach" angles that let us have a slow "final approach" to
% the actual pick and place location.  This can increase accuracy and reduce
% issues where straight-line-configuration-space trajectories make the end
% effector hit the table
position_1_approach = [-0.5192 0.8585 1 1.179 position_1(5)]';
position_2_approach = [0.3 0.7152 1 2.3 position_2(5)]';

%% Moves the robot from the initial position to the first waypoint over 4
%% seconds.  We break this into 3 seconds to make most of the motion, and 1 for
%% the final approach.
trajectory = trajectory_spline([initial_thetas midpoint position_1_approach], [0, 2, 3], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_1_approach position_1], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

%% Pick up the object at position 1.  We pause to let the robot stabilize before
%% moving. NOTE: If you pause more than the length of the command lifetime you
%% set above, then the robot "goes limp" because the previous position and/or
%% velocity and torque commands "expire".
pick(gripper);
pause(0.75);

%% Move to the second waypoint over 4 seconds, with special "retract" and
%% "approach" motions that are done more slowly.
trajectory = trajectory_spline([position_1 position_1_approach], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_1_approach midpoint position_2_approach], [0, 1, 2], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_2_approach position_2], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

%% Place the object
place(gripper);
pause(0.75);

%% Move back to position 1.
trajectory = trajectory_spline([position_2 position_2_approach], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_2_approach midpoint position_1_approach], [0, 1, 2], frequency);
command_trajectory(robot, trajectory, frequency);
trajectory = trajectory_spline([position_1_approach position_1], [0, 1], frequency);
command_trajectory(robot, trajectory, frequency);

%% Stop logging, and plot results
robot.stopLog();

hebilog = HebiUtils.convertGroupLog(fullfile(currentDir, 'robot_data.hebilog'));

% Plot angle data
figure();
subplot(3,1,1);
plot(hebilog.time, hebilog.positionCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.position, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint positions during trajectory');
xlabel('t');
ylabel('\theta');
subplot(3,1,2);
plot(hebilog.time, hebilog.velocityCmd, 'k', 'LineWidth', 1)
hold on;
plot(hebilog.time, hebilog.velocity, 'r--', 'LineWidth', 1)
hold off;
title('Plot of joint velocities during trajectory');
xlabel('t');
ylabel('joint velocities');
subplot(3,1,3);
plot(hebilog.time, hebilog.torque, 'r--', 'LineWidth', 1)
title('Plot of joint torques during trajectory');
xlabel('t');
ylabel('\tau');

end
