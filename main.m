HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
pause(.2);
G = load('jenga_gains.mat');
G.jenga_gains.positionKp = [1 1 1 1 1];
G.jenga_gains.positionKi = [0 0 0 0 0];
G.jenga_gains.positionKd = [.01 .01 .01 .01 .01];
G.jenga_gains.velocityKp = [.01 .01 .01 .01 .01];
G.jenga_gains.positionFF = [0 .5 .5 0 0]; %no idea what this is

% Connect to physical robot
robot_hardware = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});

gripper = HebiLookup.newGroupFromNames('16384','gripper');
gripper.setCommandLifetime(0);
gripper.setFeedbackFrequency(100);

robot.setCommandLifetime(1);
robot.set('gains',G.jenga_gains);
warning('Before continuing, ensure no persons or objects are within range of the robot!\nAlso, ensure that you are ready to press "ctrl-c" if the robot does not act as expected!');
disp('');
input('Once ready, press "enter" to continue...','s');

%% Setup reusable structures to reduce memory use in loop
cmd = CommandStruct();
fbk = robot_hardware.getNextFeedback();

%% Get initial position
fbk = robot_hardware.getNextFeedback();
start_theta = fbk.position' % (The transpose turns the feedback into a column vector)

%% Start logging
currentDir = fileparts(mfilename('fullpath'));
logFile = robot_hardware.startLog('file', fullfile(currentDir, 'robot_data'));


dh = [0, pi/2, .122, 0;
    .390, pi, .10355, 0;
    .334, pi, .07305, 0;
    0, pi/2, .1025, 0;
    0, 0, .100, 0];

robot=Robot(dh,[0;0;0;0;0],[0;0;1.75;0.75;0], 0);

frequency = 100;
use_velocity = true;

goal_pos = [518.4726, -86.0831, 209.6542, 0.1181, -0.9003, 3.1206].';
%goal_theta = robot.inverse_kinematics(start_theta, goal_pos);
goal_theta = [-0.405, 1.5726, 1.5659, 0.2035, -7.682].';
% goal_theta(2) = goal_theta(2) + 0.25;
robot.ee(goal_theta)

% goal_theta = start_theta;
% goal_theta(4) = goal_theta(4) + 0.5;

% waypoints = linear_joint_trajectory(start_theta, goal_theta, 3)
waypoints = [];
neutral_pos = [ 0.1342    0.7223    1.0825    1.5707   -2.3401];
%pick_pos = [0.2877, 0.7402, 1.0114, 1.7654, -2.3153];
actual_pick_pos = [  0.1342    0.5823    0.9625    1.5707   -2.3401];
actual_pick_pos2 = [  0.1342    0.6023    0.9625    1.5707   -2.3401];
actual_pick_pos3 = [  0.1342    0.6023    0.9625    1.5707   -2.3401];

pre_place_pos = [    0.9522    0.7220    1.0147    0.8009   -2.3401];
offset = 0;
a=.5*offset;
b = offset*1;
c = offset*1.2;
p1_approach = [0.8823    0.6629    1.0704    0.9475   -2.9673];
p1 = [0.8823    0.5629    1.0704    0.9475   -3.1];
p1_leaving = [0.8823    0.6629    1.0704    0.9475   -2.9673];
p2_approach = [0.96    0.6602    0.9914    0.9654   -2.9753];
p2 = [ 0.92    0.5602    0.9914    0.9654   -3.0];
p2_leaving = [0.9677    0.6602    0.9914    0.9654   -2.9753];
p3_approach = [1.0321    0.7602    1.0087    1.1355    -2.9753];
%p3 = [0.9677    0.5402+c    0.9114    1.0054   -2.7953-c];
p3 = [0.9321    0.5402    1.0087    1.1355   -2.9753];
p3_leaving = [0.9921    0.7202    1.0087    1.1355     -2.9753];
waypoints = [start_theta.'; neutral_pos; actual_pick_pos; neutral_pos; pre_place_pos;p1_approach;p1; p1_leaving;neutral_pos; actual_pick_pos2; neutral_pos; pre_place_pos; p2_approach;p2;p2_leaving; neutral_pos; actual_pick_pos3; neutral_pos; p3_approach;p3;p3_leaving;neutral_pos].';

% waypoints = linear_workspace_trajectory(robot, start_theta, goal_pos, 3)

traj_from_start = trajectory_const_vel(waypoints, 0:size(waypoints, 2), 100);
% traj_vel_from_start = diff(traj_from_start,1,2);
% 
% while true
%     
%     
% end

% 
% while true
%     fbk = robot_hardware.getNextFeedback();
%     pos = fbk.position;
%     pos
%     robot.ee(pos.')
% end

pause(2);
use_velocity = false;


for i = 1 : (size(traj_from_start, 2) - 1)
    % Send command to the robot
     cmd.position = traj_from_start(:,i)'; % transpose turns column into row vector for commands
%     cmd.position = cmd.position + offset;

%     torque = [0; 0; 0; 0; 0];
%     jacobians = robot.jacobians(traj_from_start(:,i));
%     for j = 1:5
%         jacobian = jacobians(:, :, j);
%         offset = jacobian.' * (robot.joint_masses(j) * [0; 0; -9.81; 0; 0; 0]);
%         torque = torque + offset;
%     end
%      cmd.torque = [0, 0, 0, 0, 0];

%     if (use_velocity)
%       cmd.velocity = traj_vel_from_start(:,i)' * frequency; % transpose turns column into row vector for commands
%     end
    cmd.velocity = [0, 0, 0, 0, 0];

    robot_hardware.set(cmd);
    
    fbk = robot_hardware.getNextFeedback();
    pos = fbk.position;

    
    pos

    % Wait a little bit to send at ~100Hz.
    pause(1 / frequency);
     
    
    if mod(i, 100) == 0
        for j = 1:100
         robot_hardware.set(cmd);
         pause(1/100);
        end
        
        if i == 200
            pick(gripper);
         end
         if i == 900
            pick(gripper);
         end
         if i == 1300
            place(gripper);
         end
         if i == 1600
            pick(gripper);
         end
         if i == 1900
            place(gripper);
         end
         if i == 600 %nominally 400
            place(gripper);
        
%        if i == 200
 %           pick(gripper);
  %    if i == 400 %nominally 400
    %        place(gripper);
   %     end
    end
end
% 
% i = (size(traj_from_start, 2) - 1)
% % 
% while true
%     
%     fbk = robot_hardware.getNextFeedback();
%     pos = fbk.position;
% 
%     
%     pos
%     robot.ee(pos.')
%     
% %     cmd.position = traj_from_start(:,i)' % transpose turns column into row vector for commands
% 
%     if (use_velocity)
%       cmd.velocity = traj_vel_from_start(:,i)' * frequency; % transpose turns column into row vector for commands
%  
%     end
%      cmd.torque = [0, 0.1, -0.1, 0, 0];
%     cmd.velocity = [0, 0, 0, 0, 0];
%     robot_hardware.set(cmd);
% 
%     % Wait a little bit to send at ~100Hz.
%     pause(1 / frequency);
% end
end

%function [] = pick(suction_cup)
 % suction_cmd = IoCommandStruct();
 % suction_cmd.e2 = 1;
  %suction_cup.set(suction_cmd);
%end

% Convenience function to use to hide the internal logic of stopping the suction
%function [] = place(suction_cup)
  %suction_cmd = IoCommandStruct();
  %suction_cmd.e2 = 0;
 % suction_cup.set(suction_cmd);
%end

