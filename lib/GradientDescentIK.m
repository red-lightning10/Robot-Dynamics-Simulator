function deltaQ = GradientDescentIK(J, alpha, targetPose, currentPose)
    %check and rectify if alpha is a negative number...it should be +ve
    if alpha < 0
        alpha = alpha*-1;
    end

     deltaQ = alpha*J'*(targetPose - currentPose);