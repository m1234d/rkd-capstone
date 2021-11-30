classdef Robot
    %ROBOT Represents a general fixed-base kinematic chain.
    
    properties (SetAccess = 'immutable')
        dof
        dhs
        link_masses
        joint_masses
        end_effector_mass
    end
    
    properties
        goal_position
    end
    
    methods
        % Constructor: Makes a brand new robot with the specified parameters.
        function robot = Robot(dhs, link_masses, joint_masses, end_effector_mass)
            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            
            if size(link_masses, 2) ~= 1
               error('Invalid link_masses: Should be a column vector, is %dx%d.', size(link_masses, 1), size(link_masses, 2));
            end
            
            if size(joint_masses, 2) ~= 1
               error('Invalid joint_masses: Should be a column vector.');
            end
            
            if ~isnumeric(end_effector_mass)
               error('Invalid end_effector_mass: Should be a number.'); 
            end
            
            robot.dof = size(dhs, 1);
            
            if size(dhs, 1) ~= robot.dof
                error('Invalid number of link masses: should match number of link lengths.');
            end
            
            if size(joint_masses, 1) ~= robot.dof
                error('Invalid number of joint masses: should match number of link lengths. Did you forget the base joint?');
            end
            
            robot.dhs = dhs;
            robot.link_masses = link_masses;
            robot.joint_masses = joint_masses;
            robot.end_effector_mass = end_effector_mass;  
            
            robot.goal_position = [];
        end
       
        % Returns the forward kinematic map for each frame, one for the base of
        % each link, and one for the end effector. Link i is given by
        % frames(:,:,i), and the end effector frame is frames(:,:,end).
        function frames = forward_kinematics(robot, thetas)
            if size(thetas, 2) ~= 1
                error('Expecting a column vector of joint angles.');
            end
            
            if size(thetas, 1) ~= robot.dof
                error('Invalid number of joints: %d found, expecting %d', size(thetas, 1), robot.dof);
            end
            
            % Allocate a variable containing the transforms from each frame
            % to the base frame.
            frames = zeros(4,4,robot.dof);
            n = robot.dof;
            % The transform from the base of link 'i' to the base frame (H^0_i)
            % is given by the 3x3 matrix frames(:,:,i).

            % The transform from the end effector to the base frame (H^0_i) is
            % given by the 3x3 matrix frames(:,:,end).

            as = robot.dhs(:, 1);
            alphas = robot.dhs(:, 2);
            ds = robot.dhs(:, 3);
            thetas_constant = robot.dhs(:, 4);
            thetas = thetas + thetas_constant;
            
            
            
            %% FILL IN 3x3 HOMOGENEOUS TRANSFORM FOR n + 1 FRAMES
%             frames(1, :, :) = [1, 0, 0; 0, 1, 0; 0, 0, 1];
%             frames(:, :, 1) = [1, 0, 0; 0, 1, 0; 0, 0, 1]
            frames(:, :, 1) = [cos(thetas(1)), -sin(thetas(1))*cos(alphas(1)), sin(thetas(1))*sin(alphas(1)), as(1)*cos(thetas(1));
        sin(thetas(1)), cos(thetas(1))*cos(alphas(1)), -cos(thetas(1))*sin(alphas(1)), as(1)*sin(thetas(1));
        0, sin(alphas(1)), cos(alphas(1)), ds(1);
        0, 0, 0, 1];
                
            for i = 2:n
                f = [cos(thetas(i)), -sin(thetas(i))*cos(alphas(i)), sin(thetas(i))*sin(alphas(i)), as(i)*cos(thetas(i));
        sin(thetas(i)), cos(thetas(i))*cos(alphas(i)), -cos(thetas(i))*sin(alphas(i)), as(i)*sin(thetas(i));
        0, sin(alphas(i)), cos(alphas(i)), ds(i);
        0, 0, 0, 1];
                frames(:, :, i) = frames(:, :, i-1) * f;
            end
        end
       
        % Shorthand for returning the forward kinematics.
        function fk = fk(robot, thetas)
            fk = robot.forward_kinematics(thetas);
        end
       
        function [x, y, z, psi, theta, phi] = extract(robot, H)
            x = H(1, 4);
            y = H(2, 4);
            z = H(3, 4);
            rot = H(1:3, 1:3);
            vec = rotm2eul(rot, 'ZYX');
            psi = vec(1);
            theta = vec(2);
            phi = vec(3);
        end
        
        % Returns [x; y; theta] for the end effector given a set of joint
        % angles. 
        function ee = end_effector(robot, thetas)
            % Find the transform to the end-effector frame.
            frames = robot.fk(thetas);
            H_0_ee = frames(:,:,end);
           
            % Pack them up nicely.
            [ee1, ee2, ee3, ee4, ee5, ee6] = robot.extract(H_0_ee);
            ee = [ee1, ee2, ee3, ee4, ee5, ee6].';
        end
       
        % Shorthand for returning the end effector position and orientation. 
        function ee = ee(robot, thetas)
            ee = robot.end_effector(thetas);
        end
        
        function jacobians = jacobians(robot, thetas)
            % Returns the SE(3) Jacobian for each frame (as defined in the forward
            % kinematics map). Note that 'thetas' should be a column vector.

            % Make sure that all the parameters are what we're expecting.
            % This helps catch typos and other lovely bugs.
            if size(thetas, 1) ~= robot.dof || size(thetas, 2) ~= 1
               error('Invalid thetas: Should be a column vector matching robot DOF count, is %dx%d.', size(thetas, 1), size(thetas, 2));
            end

            % Allocate a variable containing the Jacobian matrix from each frame
            % to the base frame.
            jacobians = zeros(6,robot.dof,robot.dof); 
            epsilon = 0.001;
           
            % The jacobian for frame 'i' (as defined by the forward kinematics
            % function above) is the 3xn matrix jacobians(:,:,i), where n is the
            % number of degrees of freedom.

            % In this function, compute the _numerical derivative_ for each
            % element of the Jacobian.

            % Hint: The third row of the Jacobian (the resulting frame angular
            % velocity given the joint angular velocity) is either 1 or 0,
            % depending on whether or not that joint contributes to that frame's
            % rotation.

            % Hint: jacobians(1,j,frame) is the partial differential of
            % the 'x' coordinate of 'frame' (with respect to the base frame) due
            % to motion of the joint j.  Mathematically, this is
            % d{x^0_frame}/d{theta_j}.
            %
            % To find this quantity numerically, one can compute the 'numerical
            % derivative' instead of the 'analytic derivative'.  This is
            % essentially approximating g'(x), the derivative of some function
            % g(x) at x, by computing (g(x+dx) - g(x-dx)) / (2*dx).
            %
            % Applying this to the task of finding jacobians(1,j,frame), we know
            % that:
            %   x represents theta
            %   g is the forward kinematics to this point, or the (1, 3, frame)
            %     index of the result of calling fk.
            %   dx is a vector of zeros for each joint except for joint j, and a
            %     small value for j (use the 'epsilon' defined above).
            %
            % This same logic can be used to find the y component, or
            % jacobians(2,j,frame).

            % Reminder: to get the x and y coordinates of the origin of each
            % frame, one could call the following commands (essentially pulling
            % out the translation term from the homogeneous transform to each
            % frame)
            %
            % frames = robot.fk(thetas); % Find the frames
            % frames_x = base_frames(1,3,:); % Get x values of the origins
            % frames_y = base_frames(2,3,:); % Get y values of the origins

% --------------- BEGIN STUDENT SECTION ----------------------------------
            for joint = 1 : robot.dof

                % TODO perturb the FK by 'epsilon' in this joint, and find the
                % origin of each frame.

                dx = zeros(robot.dof, 1);
                dx(joint) = epsilon;
                
                frames_high = robot.fk(thetas + dx); % Find the frames
                frames_low = robot.fk(thetas - dx);
                
                for frame = 1 : size(frames_high,3)                    
                    cur_frame_high = frames_high(:, :, frame);
                    cur_frame_low = frames_low(:, :, frame);
                    
                    [ee1, ee2, ee3, ee4, ee5, ee6] = robot.extract(cur_frame_high);
                    params_high = [ee1, ee2, ee3, ee4, ee5, ee6].';
                    
                    [ee1, ee2, ee3, ee4, ee5, ee6] = robot.extract(cur_frame_low);
                    params_low = [ee1, ee2, ee3, ee4, ee5, ee6].';
                    
                    result = (params_high - params_low) / (2*epsilon);
                    result = result.';

                    jacobians(:, joint, frame) = result;
                    
%                     if frame >= joint
%                         jacobians(3, joint, frame) = 1;
%                     else
%                         jacobians(3, joint, frame) = 0;
%                     end

%                     d = (cur_frame_high*x - cur_frame_low*x) / (2*epsilon);
%                     % TODO Fill in dx/dtheta_j for this frame
%                     jacobians(1, joint, frame) = d(1);
% 
%                     % TODO Fill in dy/dtheta_j for this frame
%                     jacobians(2, joint, frame) = d(2);
                end
            end
% --------------- END STUDENT SECTION ------------------------------------
        end
        
        function c = cost(robot, thetas)
           pos = robot.ee(thetas);
           
           jacobians = robot.jacobians(thetas);
           end_jacobian = jacobians(:, :, end);
           
           c = (pos - robot.goal_position).^2
           
%            pos
%            robot.goal_position
%            pos-robot.goal_position
%            
%            pause(1);
        end
        
        function thetas = inverse_kinematics(robot, initial_thetas, goal_position)
            % Returns the joint angles which minimize a simple squared-distance
            % cost function.

            
            robot.goal_position = goal_position;
           

            
                options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt',...
    'MaxFunctionEvaluations',10000)


%% Actually run the optimization to generate the angles to get us (close) to
%% the goal.
           thetas = lsqnonlin(@(theta) robot.cost(theta), initial_thetas, [-pi, 0, -2*pi/3, 0, -4*pi], [pi, pi/2, 2*pi/3, pi, 4*pi], options );
%             thetas = fmincon(@(theta) robot.cost(theta), initial_thetas, [0, 0, 0, 0, 0], 0, [], [], [-pi, -pi, -pi, -pi, -pi], [pi, pi, pi, pi, pi]);

%             % Allocate a variable for the joint angles during the optimization;
%             % begin with the initial condition
%             thetas = initial_thetas;
%             
%             
%             % Step size for gradient update
%             step_size = 0.1;
% 
%             % Once the norm (magnitude) of the computed gradient is smaller than
%             % this value, we stop the optimization
%             stopping_condition = 0.00005;
% 
%             % Also, limit to a maximum number of iterations.
%             max_iter = 100;
%             num_iter = 0;
% 
% % --------------- BEGIN STUDENT SECTION ----------------------------------
%             % Run gradient descent optimization
%             while (num_iter < max_iter)
% 
%                 % Compute the gradient for either an [x;y] goal or an
%                 % [x;y;theta] goal, using the current value of 'thetas'.
%                 % TODO fill in the gradient of the squared distance cost function
%                 % HINT use the answer for theory question 2, the
%                 % 'robot.end_effector' function, and the 'robot.jacobians'
%                 % function to help solve this problem
%                 if (size(goal_position, 1) == 2) % [x;y] goal                    
%                     jacobians = robot.jacobians(thetas);
%                     end_jacobian = jacobians(:, :, end);
%                     
%                     pos = robot.end_effector(thetas);
%                     
%                     xy_pos = [pos(1); pos(2); 0];
%                     goal_pos = [goal_position(1); goal_position(2); 0];
%                     cost_gradient = end_jacobian.' * (xy_pos - goal_pos);
%                 else % [x;y;theta] goal
%                     jacobians = robot.jacobians(thetas);
%                     end_jacobian = jacobians(:, :, end);
%                     
%                     pos = robot.end_effector(thetas);
% 
%                     cost_gradient = end_jacobian.' * (pos - goal_position);
%                 end
% 
%                 % Update 'thetas'
%                 % TODO
%                 thetas = thetas - step_size * cost_gradient;
% 
%                 % Check stopping condition, and return if it is met.
%                 % TODO
%                 
%                 if norm(cost_gradient) < stopping_condition
%                     break
%                 end
% 
%                 num_iter = num_iter + 1;
%             end
% --------------- END STUDENT SECTION ------------------------------------
        end

    end
end