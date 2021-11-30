function [ trajectory ] = trajectory_trap_vel(waypoints, times, frequency, duty_cycle)
% trajectory_const_vel
%
%   Returns a matrix of joint angles, where each column represents a single
%   timestamp. These joint angles form trapezoidal velocity trajectory segments,
%   hitting waypoints(:,i) at times(i).
%
%   'waypoints' is a matrix of waypoints; each column represents a single
%   waypoint in joint space, and each row represents a particular joint.
%
%   'times' is a row vector that indicates the time each of the waypoints should
%   be reached.  The number of columns should equal the number of waypoints, and
%   should be monotonically increasing.  The first number is technically
%   arbitrary, as it serves only as a reference for the remaining timestamps,
%   but '0' is a good default.
%
%   'frequency' is the control frequency which this trajectory should be played
%   at, and therefore the number of columns per second of playback.

% Number of joints:
num_joints = size(waypoints, 1);
% Number of waypoints:
num_waypoints = size(waypoints, 2);
% Number of segments between waypoints:
num_segments = num_waypoints - 1;

if (size(times) ~= [1, num_waypoints])
    error('Size of times vector is incorrect!');
end

if (num_waypoints < 2)
    error('Insufficient number of waypoints.');
end

if (length(frequency) ~= 1 || frequency < 5)
    error('Invalid control frequency (must be at least 5Hz)');
end

if (duty_cycle < 0 || duty_cycle > 0.5)
    error('Invalid duty cycle!');
end

%% First, we go through each segment between waypoints, and calculate how many
%% points we must generate in this segment given our control frequency.
num_points_per_segment = zeros(1, num_segments);
for segment = 1:num_segments
    dt = times(segment+1) - times(segment);
    num_points_per_segment(segment) = dt * frequency;
end

%% Go through each segment of the trajectory, and fill it in segment-by-segment.

% Pre-allocate a trajectory matrix with the correct number of rows and columns.
trajectory = zeros(num_joints, sum(num_points_per_segment));

% The index of the first point in this segment
segment_start_point = 1;
for segment = 1:num_segments
    % Number of points in this segment
    points_in_segment = num_points_per_segment(segment);

    % The index of the last point in this segment
    segment_end_point = segment_start_point + points_in_segment - 1;

    % Find the number of points of the ramp up (and also of the ramp down)
    % section of the trajectory
    num_ramp_points = floor(duty_cycle * points_in_segment);

    % The length of the ramp up time (and also ramp down time), in seconds.
    ramp_time = (times(segment+1) - times(segment)) * duty_cycle;

    % --------------- BEGIN STUDENT SECTION ----------------------------------
    
    start_theta = waypoints(:, segment);
    end_theta = waypoints(:, segment+1);
    
    % Find the maximum velocity through this segment, based on the duty cycle
    vm = zeros(num_joints, 1); % fill this in! 
    for j = 1:size(vm, 1)
        vm(j) = (end_theta(j) - start_theta(j))/(times(segment+1) - times(segment) - ramp_time);
    end

    % Fill in the points in this segment of the trajectory.  Those points will
    % be:
    % trajectory(:, segment_start_point : segment_start_point + num_ramp_points - 1)              % ramp up
    % trajectory(:, segment_start_point + num_ramp_points : segment_end_point - num_ramp_points)  % constant velocity
    % trajectory(:, segment_end_point - num_ramp_points + 1 : segment_end_points)                 % ramp down
    for j = 1:size(start_theta, 1)
        t = times(segment);
        dt = (times(segment+1) - times(segment)) /( points_in_segment - 1);
        t0 = times(segment);
        tf = times(segment+1);
        trajectory(j, segment_start_point) = start_theta(j);
        t = t + dt;
        for i = segment_start_point+1 : segment_start_point + num_ramp_points
            trajectory(j, i) = start_theta(j) + (vm(j) / (2*ramp_time))*((t - t0)^2);
            t = t + dt;
        end
        
        ta = t0 + ramp_time;
        qta = start_theta(j) + (vm(j) / (2*ramp_time))*((ta - t0)^2);
        for i = segment_start_point + num_ramp_points+1 : segment_end_point - num_ramp_points
            trajectory(j, i) = qta + vm(j)*(t - ta);
            t = t + dt;
        end
        
        tb = tf - ramp_time;
        qtb = qta + vm(j)*(tb - ta);
        for i = segment_end_point - num_ramp_points + 1 : segment_end_point
            trajectory(j, i) = qtb - (vm(j) / (2*ramp_time))*(tf^2 - 2*tf*t + t^2 - ramp_time^2);
            t = t + dt;
        end        
    end
    % NOTE: it will be useful to default some helper variables here to match
    % the equations from the background material, and potentially loop through
    % the trajectory joint-by-joint and point-by-point to fill in each element
    % directly unless you are familiar with MATLAB vector operations.

    % --------------- END STUDENT SECTION ------------------------------------

    % Update the starting index as we move to the next segment.
    segment_start_point = segment_start_point + points_in_segment;
end

end
