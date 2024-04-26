function q = ikin(S, M, currentQ, targetP)
    cycle = 1;
    func = @(theta) norm(targetP - calculate_position(theta, S, M));
    % sol = fsolve(func, currentQ);
    lb = deg2rad([-180, -180, -180,-180,-180,-180])';
    ub = deg2rad([180, 180, 180, 180, 180, 180])';
    %supressing annoying output from this solver
    options = optimoptions('fmincon', 'Display', 'off');
    %using constraints on solver to get solution within joint limits
    problem.objective = func;
    problem.x0 = currentQ;
    problem.lb = lb;
    problem.ub = ub;
    problem.solver = 'fmincon';
    problem.options = options;

    [sol, objf, cvg, outp]= fmincon(problem);
    q = sol;
    % currentP = M(1:3,4);
    % alpha = 0.12;
%     while norm(targetP - currentP) > 1e-6 && cycle < 100
%             J = jacoba(S, M, currentQ);
%             deltaQ = GradientDescentIK(J, alpha, targetP, currentP);
%             currentQ = currentQ + deltaQ';
%             T = fkine(S,M,currentQ, 'space');
%             currentP = T(1:3,4);
%             cycle = cycle + 1;
%     end
% 
%     if norm(targetP - currentP) <= 1e-6
%         disp("Answer close enough")
%         cond1 = 1;
%     else
%         disp("Not enough")
%         cond1 = 0;
%     end
%     q = sol;
%     if checkJointLimits(q)
%         disp("Answer obeys joint limits")
%         cond2 = 1;
%     else
%         disp("Answer does not fall within the joint limits")
%         cond2 = 0;
%         while norm(targetP - currentP) > 1e-2
%             J = jacoba(S, M, currentQ);
%             deltaQ = GradientDescentIK(J, alpha, targetP, currentP);
%             currentQ = currentQ + deltaQ';
%             T = fkine(S,M,currentQ, 'space');
%             currentQ = currentQ + deltaQ';
%             sol = fsolve(func, currentQ);
%             currentP = calculate_position(sol, S, M);
%         end
%     end
%     q = sol;
% end
% 
function currentP = calculate_position(currentQ, S, M)
    T = fkine(S,M,currentQ, 'space');
    currentP = T(1:3,4);
end
% 
% function var = checkJointLimits(q)
%     joint_limits_l = deg2rad([-180, -180, -180,-180,-180,-180]);
%     joint_limits_r = deg2rad([180, 180, 180, 180, 180, 180]);
%     if all(q > joint_limits_l) && all(joint_limits_r > q)
%         var = 1;
%     else
%         var = 0;
%     end

end