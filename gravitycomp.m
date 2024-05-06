function [tau_acc,jointPos_acc,jointVel_acc,jointAcl_acc, t_acc] = gravity_comp(curr_Q, S, M, Mlist, Glist, g, n, Ftip)
    tau_acc = [];
    jointPos_acc = [];
    t_acc = [];
    jointAcl_acc = [];
    jointVel_acc = [];

    %% Gravity Compensation
    fprintf('-----------------------Gravity Compensation-----------------------\n');
    % We are now going to solve the inverse dynamics and calculate the torques
    % required to keep the robot where it is.
    
    % Initialize the parameters for the RNE algorithm
    clear params
    
    params.g = g; % gravity vector
    params.S = S; % screw axes
    params.M = Mlist; % link frames 
    params.G = Glist; % inertial properties
    params.jointPos = curr_Q; % desired joint positions
    if size(curr_Q,1) == 1
        params.jointPos = curr_Q';
    end
    params.jointVel = zeros(6,1); % desired joint velocities
    params.jointAcc = zeros(6,1); % desired joint accelerations
    % params.Ftip = Ftip;     % desired wrench at the end effector
    T = fkine(S, M, params.jointPos, 'space');
    ext_wrench = [cross(T(1:3, 4), -Ftip(4:6)); -Ftip(4:6)]; 
    params.Ftip = adjoint(T)'*ext_wrench; % end effector wrench
    % Invoke the RNE algorithm to calculate the joint torques needed for
    % gravity compensation
    tau = rne(params);
    % fprintf('Joint Torques: ');
    % fprintf('[%f %f %f] Nm\n', tau(1), tau(2), tau(3), tau(4), tau(5), tau(6));
    
    % To make sure that the solution is correct, let us simulate the robot
    % fprintf('\nWe are now going to simulate the robot to see if it moves.\n');
    fprintf('Calculating the Forward Dynamics: ');
    nbytes = fprintf('0%%');
    
    dt = 1e-1;        % simulation time step [s]
    t = 0 : dt : 0.2; % total simulation time [s]
    
    qt = zeros(n,size(t,2));  qt(:,1) = params.jointPos;
    qdt = zeros(n,size(t,2)); qdt(:,1) = params.jointVel;
       
    for ii = 1 : size(t,2) - 1
        fprintf(repmat('\b',1,nbytes));
        nbytes = fprintf('%3.0f%%', 100*(ii/(size(t,2)-1)));
    
        % Set torques
        params.tau = tau; % supply the torques needed for gravity compensation
        T = fkine(S, M, params.jointPos, 'space');
        ext_wrench = [cross(T(1:3, 4), -Ftip(4:6)); -Ftip(4:6)]; 
        params.Ftip = adjoint(T)'*ext_wrench; % end effector wrench
        % Calculate the joint accelerations
        jointAcc = fdyn(params);
        disp(ext_wrench)
        % Integrate the joint accelerations to get velocity and
        % position
        params.jointPos = params.jointPos + dt * params.jointVel;
        params.jointVel = params.jointVel + dt * jointAcc;
    
        % Accumulate results
        qt(:,ii+1) = params.jointPos;
        qdt(:,ii+1) = params.jointVel;
        jointAcl_acc = [jointAcl_acc jointAcc];
    end
        jointAcl_acc = [jointAcl_acc jointAcc];
        jointVel_acc = [jointVel_acc qdt];
        tau_acc = [tau_acc tau];
        jointPos_acc = [jointPos_acc qt];
        t_acc = [t_acc 1 : size(t,2)];

end