function ddq = omni_robot_model(q, dq, tf, m1, m2, r, L)
    [A1, A2, b, M11, M12, M21, M22, F1, F2, K, X] = cal_basic_var_withF(q, dq, m1, m2, r, L);
    ddq1 = A1^(-1)*b - K*X^(-1)*(F2 + K'*M11*A1^(-1)*b - K'*(F1+tf));
    ddq2 = X^(-1)*(F2 + K'*M11*A1^(-1)*b - K'*(F1+tf));
    ddq = [ddq1; ddq2];
end
