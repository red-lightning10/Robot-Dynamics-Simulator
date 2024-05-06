function [tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = movementDynamics(currentQ, curr_vel, curr_acc, S, M, Mlist, Glist, g, n, waypoints, Ftip)
    fprintf('Generating the Trajectory and Torque Profiles... ');
    nbytes = fprintf('0%%');
    nPts = size(waypoints,2);
    tau_acc = [];
    jointPos_acc = [];
    t_acc = [];
    jointAcl_acc = [];
    jointVel_acc = [];

    for jj = 1 : nPts - 1
        fprintf(repmat('\b',1,nbytes));
        nbytes = fprintf('%3.0f%%', 100*(jj/(nPts - 1)));
       
        % Initialize the time vector
        dt = 5e-2;       % time step [s]
        t  = 0 : dt : 1; % total time [s]
    
        % Initialize the arrays where we will accumulate the output of the robot
        % dynamics
        jointPos_prescribed = zeros(n,size(t,2)); % Joint Variables (Prescribed)
        jointVel_prescribed = zeros(n,size(t,2)); % Joint Velocities (Prescribed)
        jointAcc_prescribed = zeros(n,size(t,2)); % Joint Accelerations (Prescribed)
        tau_prescribed      = zeros(n,size(t,2)); % Joint Torques
    
        jointPos_actual = zeros(n,size(t,2)); % Joint Variables (Actual)
        jointVel_actual = zeros(n,size(t,2)); % Joint Velocities (Actual)
    
        % For each joint
        for ii = 1 : n
            % Calculate a trajectory using a quintic polynomial
            params_traj.t = [0 t(end)]; % start and end time of each movement step
            params_traj.time_step = dt;
            params_traj.q = [waypoints(ii,jj) waypoints(ii,jj+1)];
    
            params_traj.v = [0 0];
            params_traj.a = [0 0];
    
            traj = make_trajectory('quintic', params_traj);
    
            % Generate the joint profiles (position, velocity, and
            % acceleration)
            jointPos_prescribed(ii,:) = traj.q;
            jointVel_prescribed(ii,:) = traj.v;
            jointAcc_prescribed(ii,:) = traj.a;
        end
        
        % Initialize the parameters for both inverse and forward dynamics
        params_rne.g = g; % gravity
        params_rne.S = S; % screw axes
        params_rne.M = Mlist; % link frames
        params_rne.G = Glist; % inertial properties
        params_fdyn.g = g; % gravity
        params_fdyn.S = S; % screw axes
        params_fdyn.M = Mlist; % link frames
        params_fdyn.G = Glist; % inertial properties
    
    
        % Initialize the (actual) joint variables
        jointPos_actual(:,1) = jointPos_prescribed(:,1);
        jointVel_actual(:,1) = jointVel_actual(:,1);
    
        for ii = 1 : size(t,2) - 1
            % Calculate the joint torques using the RNE algorithm
            params_rne.jointPos = jointPos_prescribed(:,ii);
            params_rne.jointVel = jointVel_prescribed(:,ii);
            params_rne.jointAcc = jointAcc_prescribed(:,ii);
            % Ftip = zeros(6,1);
            T = fkine(S, M, jointPos_prescribed(:,ii), 'space');
            ext_wrench = [cross(T(1:3, 4), -Ftip(4:6)); -Ftip(4:6)]; 
            params_rne.Ftip = adjoint(T)'*ext_wrench; % end effector wrench
            %params_rne.Ftip = Ftip; % end effector wrench
    
            tau_prescribed(:,ii) = rne(params_rne);
    
            % Feed the torques to the forward dynamics model and perform one
            % simulation step
            params_fdyn.jointPos = jointPos_actual(:,ii);
            params_fdyn.jointVel = jointVel_actual(:,ii);
            params_fdyn.tau = tau_prescribed(:,ii);
            T = fkine(S, M, jointPos_actual(:,ii), 'space');
            ext_wrench = [cross(T(1:3, 4), -Ftip(4:6)); -Ftip(4:6)]; 
            params_fdyn.Ftip = adjoint(T)'*ext_wrench; % end effector wrench
            % params_fdyn.Ftip = Ftip; % end effector wrench
    
            jointAcc = fdyn(params_fdyn);
    
            % Integrate the joint accelerations to get velocity and
            % position
            jointVel_actual(:,ii+1) = dt * jointAcc + jointVel_actual(:,ii);
            jointPos_actual(:,ii+1) = dt * jointVel_actual(:,ii) + jointPos_actual(:,ii);
            jointAcl_acc = [jointAcl_acc jointAcc];
        end
    
        tau_prescribed(:,end) = tau_prescribed(:,end-1);
        jointVel_acc = [jointVel_acc jointVel_actual];
        
        tau_acc = [tau_acc tau_prescribed];
        jointPos_acc = [jointPos_acc jointPos_actual];
        t_acc = [t_acc t+t(end)*(jj)];
        jointAcl_acc = [jointAcl_acc jointAcl_acc(:,end)];
    end
    
    fprintf('\nIteration done');
end