function tf = cf_controller(qd, dqd, ddqd, q, dq, m1, m2, r, L)
    [A1, A2, b, M11, M12, M21, M22, F1, F2, K, X] = cal_basic_var_withoutF(q, dq, m1, m2, r, L);
    % 可调参数
    S = diag([1,1,1]);
    %gamma = 0.01;
    gamma =0.01;
    P = diag([1,1,1]);
    q2    = q(4:6);
    q2d   = qd(4:6);
    dq2   = dq(4:6);
    dq2d  = dqd(4:6);
    ddq2d = ddqd(4:6);
    % 系统参数
    beta = S*(dq2 - dq2d) + (q2 - q2d);
    % 控制器
    p1 = (K')^(-1)*(F2+K'*M11*A1^(-1)*b-X*(ddq2d+S*dq2d-S*dq2)) - F1;
    p2 = gamma*X^(-1)*(K')^(-1)*P^(-1)*beta;
    tf = p1 + p2;
    tfmax = 1.5;
    tf = max(min(p1 + p2, [tfmax;tfmax;tfmax]),[-tfmax;-tfmax;-tfmax]);
end
