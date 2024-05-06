function createRobotSimulatorApp()

addpath('lib');
global robot currentQ robotAxes robotPanel plotPanel posXCheckbox posYCheckbox posZCheckbox externalForceField externalPathField Ftip path;
global f axesPositions axesVelocities axesAccelerations axesTorques
% Main figure
f = figure('Name', 'Awesome PUMA 560 Simulator', 'NumberTitle', 'off', ...
    'MenuBar', 'none', 'ToolBar', 'none', 'Position', [100, 120, 1024, 768]);

% UI panels
robotPanel = uipanel('Parent', f, 'Title', 'Robot- PUMA 560', 'Position', [0.05, 0.2, 0.6, 0.75]);
plotPanel = uipanel('Parent', f, 'Title', 'Robot Profile', 'Position', [0.65, 0.05, 0.32, 0.9]);
controlPanel = uipanel('Parent', f, 'Title', 'Control Panel', 'Position', [0.05, 0.05, 0.6, 0.15]);
% dataPanel = uipanel('Parent', f, 'Title', 'Simulation Data', 'Position', [0.7, 0.05, 0.25, 0.9]);

axesPositions = axes('Parent', plotPanel, 'Position', [0.1, 0.8, 0.8, 0.17]);
axesVelocities = axes('Parent', plotPanel, 'Position', [0.1, 0.55, 0.8, 0.17]);
axesAccelerations = axes('Parent', plotPanel, 'Position', [0.1, 0.3, 0.8, 0.17]);
axesTorques = axes('Parent', plotPanel, 'Position', [0.1, 0.05, 0.8, 0.17]);

% Axes for robot visualization

robotAxes = axes('Parent', robotPanel, 'Position', [0.05, 0.1, 0.9, 0.85]);

% Create and plot the robot
robot = make_robot();
robot.plot(zeros(1,6));
currentQ = zeros(1,6);
% Control checkboxes
posXCheckbox = uicontrol('Parent', controlPanel, 'Style', 'checkbox', 'String', 'FX', 'Position', [10, 10, 50, 25]);
posYCheckbox = uicontrol('Parent', controlPanel, 'Style', 'checkbox', 'String', 'FY', 'Position', [70, 10, 50, 25]);
posZCheckbox = uicontrol('Parent', controlPanel, 'Style', 'checkbox', 'String', 'FZ', 'Position', [130, 10, 50, 25]);

% Create labels for each parameter
labels = {'X:(m)','Y:(m)', 'Z:(m)', 'Roll:(degrees)', 'Pitch:(degrees)', 'Yaw:(degrees)'};
numLabels = numel(labels);
for i = 1:numLabels
    uicontrol('Parent', controlPanel, 'Style', 'text', 'String', labels{i}, ...
        'Position', [(i-1) * 100,60 , 100, 25]);
end

% Create text boxes for each parameter
textBoxPositions = [0, 40;100, 40; 200, 40; 300, 40; 400, 40; 500, 40]; % Adjust positions as needed
for i = 1:numLabels
    uicontrol('Parent', controlPanel, 'Style', 'edit', 'String', '0', ...
        'Position', [textBoxPositions(i, 1), textBoxPositions(i, 2), 100, 25], ...
        'Callback', @(src, event) externalPathCallback(src, event, i));
end

% External force label and field
uicontrol('Parent', controlPanel, 'Style', 'text', 'String', 'External Force:(N)', 'Position', [190, 10, 100, 25]);
externalForceField = uicontrol('Parent', controlPanel, 'Style', 'edit', 'String', '0', ...
    'Position', [300, 10, 100, 25], 'Callback', @externalForceCallback);

% Button to animate robot
uicontrol('Parent', controlPanel, 'Style', 'pushbutton', 'String', 'Animate Robot', ...
    'Position', [410, 10, 120, 25], 'Callback', @animateRobot);
Ftip = zeros(6,1);

    
end
function externalPathCallback(src, event, index)
% Callback function for text boxes
global posXCheckbox posYCheckbox posZCheckbox robotAxes robotPanel Ftip path;

% Display the parameter index
disp(['Value entered for parameter ', num2str(index)]);

% Initialize path if not already initialized
if isempty(path)
    path = zeros(6,1);  % Assuming 6 parameters as a typical case
end

% Get the numeric value from the text box
newValue = str2double(get(src, 'String'));

% Validate the input to ensure it is a number
if isnan(newValue)
    % Reset the text box if the input is not valid
    set(src, 'String', '0');
    disp('Invalid input: Please enter a numeric value.');
else
    % Update the path array at the specified index
    path(index) = newValue;
    % Display the updated value for debugging
    disp(['New value set at path(', num2str(index), '): ', num2str(path(index))]);
    % Optionally display the entire path array
    disp('Updated path values:');
    disp(path)
end
end
function externalForceCallback(src, event)
global posXCheckbox posYCheckbox posZCheckbox robotAxes robotPanel Ftip;
forceValue = str2double(src.String);
if isnan(forceValue) || forceValue < 0
    src.String = '0'; % Reset to default
    disp(['External force set to Zero']);
    Ftip = zeros(6,1);
else
    disp(['External force set to ', num2str(forceValue)]);
    Ftip = zeros(6,1);

    % Check state of position checkboxes and respond accordingly
    if get(posXCheckbox, 'Value')
        disp('X position is checked.');
        Ftip(4) = forceValue * 9.81;
        % Additional logic for X
    end
    if get(posYCheckbox, 'Value')
        disp('Y position is checked.');
        Ftip(5) = forceValue * 9.81;
        % Additional logic for Y
    end
    if get(posZCheckbox, 'Value')
        disp('Z position is checked.');
        Ftip(6) = forceValue * 9.81;
        % Additional logic for Z
    end
    % robotAxes = axes('Parent', robotPanel, 'Position', [0.05, 0.1, 0.9, 0.85]);
    % cla(robotAxes);
    % [robot,tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = robot_stack(Ftip);
    % plot_function(tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc)
    % Assume applyForceAtTip handles the physical force application logic
    % Just an example, adjust as needed

end
end


% function make_robot_callback(src,event)
%     robot = make_robot();
%     robot.plot(zeros(1,6));
% end
function animateRobot(src, event)
global currentQ robotAxes robot robotPanel Ftip path f;
disp(path)
g = [0 0 -9.81]';
curr_vel = zeros(6,1);
curr_acc = zeros(6,1);
[S,M] = make_kinematics_model(robot);
n = size(S,2);
[Mlist,Glist] = make_dynamics_model(robot);
path_const = path;
curr_Q = currentQ;
if size(curr_Q,2) == 1
    curr_Q = curr_Q';
end
robot.plot(curr_Q);
while (true)

    disp('animating')
    T = [eul2rotm(path_const(4:6)') path_const(1:3); 0 0 0 1];
    targetQ = robot.ikine(T)';
    if size(curr_Q,1) == 1
        curr_Q = curr_Q';
    end
    T_curr = fkine(S,M,curr_Q, "space");
    if norm(T_curr(1:3,4) - path_const(1:3)) < 0.1
        disp('grav comp')
        [tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = gravitycomp(curr_Q, S, M, Mlist, Glist, g, n, Ftip);
    else
        waypoints(:,1) = curr_Q;
        waypoints(:,2) = targetQ;

        [tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = movementDynamics(curr_Q, curr_vel, curr_acc, S, M, Mlist, Glist, g, n, waypoints, Ftip);
        % [tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = gravitycomp(curr_Q, S, M, Mlist, Glist, g, n, Ftip);
    end


    % [tau_acc, jointPos_acc, jointVel_acc, jointAcl_acc, t_acc] = robot_stack(Ftip, path); % Assuming this function calculates forward dynamics
    % currentQ = jointPos_Acc(:, end);
    % Update GUI or perform other tasks here
    disp(['Forward dynamics calculated at ', datestr(now)]);
    % scatter3(path(1,:), path(2,:), path(3,:), 'filled');
    disp('here')
    % robotAxes = axes('Parent', robotPanel, 'Position', [0.05, 0.1, 0.9, 0.85]);
    set(f, 'currentaxes', robotAxes);
    % cla(robotAxes);
    robot.plot(jointPos_acc(:,1:end)');
    curr_Q = jointPos_acc(:, end);
    currentQ = curr_Q;
    curr_vel = jointVel_acc(:,end);
    curr_acc = jointAcl_acc(:,end);
    % disp(norm(T_curr(1:3,4) - path(1:3)))
    % disp(size(jointPos_acc))
    % disp(size(jointVel_acc))
    % disp(size(jointAcl_acc))
    % disp(size(t_acc))
    % robotAxes = axes('Parent', robotPanel, 'Position', [0.05, 0.1, 0.9, 0.85]);
    plot_function(tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc);
    % % plot_function(tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc);
    % Pause for the specified interval
    % pause(0.5);
end

% [robot,tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = robot_stack(Ftip, path);
% nPts = 100;
%
% path = sprial(nPts);

% scatter3(path(1,:), path(2,:), path(3,:), 'filled');
% robot.plot(jointPos_acc(:,1:100:end)','trail',{'r', 'LineWidth', 2});
% plot_function(tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc);
    
end



function plot_function(tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc)

global plotPanel axesTorques axesAccelerations axesVelocities axesPositions f;

% Plotting the joint positions on the  axesPositions

% axesPositions = axes('Parent', plotPanel, 'Position', [0.1, 0.75, 0.8, 0.2]);
% axesVelocities = axes('Parent', plotPanel, 'Position', [0.1, 0.5, 0.8, 0.2]);
% axesAccelerations = axes('Parent', plotPanel, 'Position', [0.1, 0.25, 0.8, 0.2]);
% axesTorques = axes('Parent', plotPanel, 'Position', [0.1, 0.05, 0.8, 0.2]);

% cla(robotAxes);
% cla(axesPositions)
% cla(axesVelocities)
% cla(axesAccelerations)
% cla(axesTorques)

set(f, 'currentaxes', axesPositions);
plot(axesPositions, t_acc, jointPos_acc(1,:), 'LineWidth', 2);
hold(axesPositions, 'on');
for j = 2:size(jointPos_acc, 1)
    plot(axesPositions, t_acc, jointPos_acc(j,:), 'LineWidth', 2);
end
hold( axesPositions, 'off');
title( axesPositions, 'Joint Positions');
legend( axesPositions, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
legendAdjust();
xlabel( axesPositions, 'Time [s]'), ylabel( axesPositions, 'Joint Position');
set( axesPositions, 'FontSize', 8);

% Plotting the joint velocities on the  axesVelocities
set(f, 'currentaxes', axesVelocities)
plot(axesVelocities, t_acc, jointVel_acc(1,:), 'LineWidth', 2);
hold(axesVelocities, 'on');
for j = 2:size(jointVel_acc, 1)
    plot(axesVelocities, t_acc, jointVel_acc(j,:), 'LineWidth', 2);
end
hold( axesVelocities, 'off');
title( axesVelocities, 'Joint Velocities');
legend( axesVelocities, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
legendAdjust();
xlabel(axesVelocities, 'Time [s]'), ylabel( axesVelocities, 'Joint Velocities ');
set(axesVelocities, 'FontSize', 8);


% Plotting the joint accelerations on the  axesAccelerations
set(f, 'currentaxes', axesAccelerations)
plot(axesAccelerations, t_acc, jointAcl_acc(1,:), 'LineWidth', 2);
hold(axesAccelerations, 'on');
for j = 2:size(jointAcl_acc, 1)
    plot(axesAccelerations, t_acc, jointAcl_acc(j,:), 'LineWidth', 2);
end
hold(axesAccelerations, 'off');
title(axesAccelerations, 'Joint Accelerations');
legend(axesAccelerations, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
legendAdjust();
xlabel( axesAccelerations, 'Time [s]'), ylabel(axesAccelerations, 'Joint Accelerations');
set(axesAccelerations, 'FontSize', 8);

% Plotting the torque profiles on the axesTorques
set(f, 'currentaxes', axesTorques)
plot(axesTorques, t_acc, tau_acc(1,:), 'LineWidth', 2);
hold(axesTorques, 'on');
for j = 2:size(tau_acc, 1)
    plot(axesTorques, t_acc, tau_acc(j,:), 'LineWidth', 2);
end
hold(axesTorques, 'off');
title(axesTorques, 'Joint Torques');
legend(axesTorques, {'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
legendAdjust();
xlabel(axesTorques, 'Time [s]'), ylabel(axesTorques, 'Torque [Nm]');
set(axesTorques, 'FontSize', 8);

function legendAdjust()
        h = legend;
        set(h, 'FontSize', 5);  % Set font size
        set(h, 'FontWeight', 'normal');  % Set font weight (e.g., 'normal', 'bold')
        set(h, 'LineWidth', 0.5);  % Set box line width
        set(h, 'Box', 'off');
end

end

function startSimulationCallback(src, event)
disp('Starting simulation...');
createRobotSimulatorApp();
% Here you would typically call a function to start the simulation
end
