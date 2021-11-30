close all
HebiLookup.setLookupAddresses('*');
HebiLookup.clearModuleList();
HebiLookup.clearGroups();
pause(3);
G = load('jenga_gains.mat');
G.jenga_gains.positionKp = [1 1 1 1 1];
G.jenga_gains.positionKi = [0 0 0 0 0];
G.jenga_gains.positionKd = [.01 .01 .01 .01 .01];
G.jenga_gains.velocityKp = [.01 .01 .01 .01 .01];
G.jenga_gains.positionFF = [0 .5 .5 0 0]; %no idea what this is
robot = HebiLookup.newGroupFromNames('16384',{'base','shoulder','elbow','wrist1','wrist2'});
robot.setCommandLifetime(1);
robot.set('gains',G.jenga_gains);

global gripper
gripper = HebiLookup.newGroupFromNames('16384','gripper');
gripper.setCommandLifetime(0);
gripper.setFeedbackFrequency(100);

global known_path
known_path = load('shikata').path;
global index
index = 1;
global delta
delta = .02;
global timer
timer = 0;

%len = size(known_path,1);

input('press enter to begin')



%should put gripper init here
fbk = robot.getNextFeedback();
global goal_thetas;
goal_thetas = fbk.position;


figgy = figure();
set(figgy,'WindowKeyPressFcn',@keypress)

while true
    if goal_thetas(1) ~= -1
        cmd = CommandStruct();
        cmd.position = goal_thetas;
        cmd.velocity = [0 0 0 0 0];
        robot.set(cmd);
    end
    pause(0.01)
    timer = timer +0.01;
end


function keypress(source,eventdata)
global known_path
global index
global delta
global goal_thetas;
global gripper
key = eventdata.Key;
%disp(key);
switch(key)
    case'q'
        goal_thetas(1) = goal_thetas(1) +delta;
    case'w'
        goal_thetas(2) = goal_thetas(2) +delta;
    case'e'
        goal_thetas(3) = goal_thetas(3) +delta;
    case'r'
        goal_thetas(4) = goal_thetas(4) +delta;
    case't'
        goal_thetas(5) = goal_thetas(5) +delta;
    case'a'
        goal_thetas(1) = goal_thetas(1) -delta;
    case's'
        goal_thetas(2) = goal_thetas(2) -delta;
    case'd'
        goal_thetas(3) = goal_thetas(3) -delta;
    case'f'
        goal_thetas(4) = goal_thetas(4) -delta;
    case'g'
        goal_thetas(5) = goal_thetas(5) -delta; 
    case'leftarrow'
        index = max(1,index-1);
        goal_thetas = known_path(index,:);
    case'rightarrow'
        index = min(size(known_path,1),index+1);
        goal_thetas = known_path(index,:);
    case 'return'
        disp(goal_thetas)
    case '1'
        known_path(end+1,:) = goal_thetas;
    case '2'
        known_path = known_path(end-1,:);
    case '3'
        known_path = [];
    case '0'
        path = known_path;
        save('shikata','path')
    case 'numpad8'
        [1 2]*[8 9];
    case 'o'
        pick(gripper);
    case 'p'
        place(gripper);
    case j
        main
        
end

end

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