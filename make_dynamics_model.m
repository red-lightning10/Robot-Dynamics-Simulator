function [Mlist,Glist] = make_dynamics_model(robot)
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

%% Link poses when the robot is in the home configuration
[M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(robot);
Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

m1 =3.7;
m2 = 8.393;
m3 = 2.275;
m4 = 1.219;
m5 = 1.219;
m6 = 0.1879;

%% Spatial Inertia Matrices
% *** Define the link inertial properties ***

G1 = zeros(6,6); 
G2 = zeros(6,6); 
G3 = zeros(6,6); 
G4 = zeros(6,6); 
G5 = zeros(6,6); 
G6 = zeros(6,6); 


G1(1:3,1:3) = diag([0.010267495893 0.010267495893 0.00666]);
G1(4:6,4:6) = m1 * eye(3);

G2(1:3,1:3) = diag([0.22689067591 0.22689067591 0.0151074]);
G2(4:6,4:6) = m2 * eye(3);

G3(1:3,1:3) = diag([0.049443313556 0.049443313556 0.004095]);
G3(4:6,4:6) = m3 * eye(3);

G4(1:3,1:3) = diag([0.111172755531 0.111172755531 0.21942]);
G4(4:6,4:6) = m4 * eye(3);

G5(1:3,1:3) = diag([0.111172755531 0.111172755531 0.21942]);
G5(4:6,4:6) = m5 * eye(3);

G6(1:3,1:3) = diag([0.0171364731454 0.0171364731454 0.033822]);
G6(4:6,4:6) = m6 * eye(3);

Glist = cat(3, G1, G2, G3, G4, G5, G6);

end