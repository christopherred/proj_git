function [A1, A2, b, M11, M12, M21, M22, F1, F2, K, X] = cal_basic_var(q, dq, m1, m2, r, L)
    % m1;             % 单个轮子的质量
    % m2;             % 中心体的质量
    % I1;             % 单个轮子的惯性矩
    % I2;             % 中心体的惯性矩
    % r;              % 单个轮子的半径
    % L;              % 中心体的半径
    [phi1,   phi2,   phi3,   x,   y,   theta]   = deal(q(1),q(2),q(3),q(4),q(5),q(6));
    [dphi1,  dphi2,  dphi3,  dx,  dy,  dtheta]  = deal(dq(1),dq(2),dq(3),dq(4),dq(5),dq(6));
    I1                   = 0.5*m1*r*r;
    I2                   = 0.5*m2*L*L;
    [A1, A2]             = cal_A();
    b                    = cal_b();
    [M11, M12, M21, M22] = cal_M();
    [F1, F2]             = cal_F();
    K                    = cal_K();
    X                    = cal_X();


    function [A1, A2] = cal_A()
        A1 = diag([-r, -r, -r]);
        A2 = [   sin( theta + pi/3 ), -cos( theta + pi/3 ), -L;
                    -sin( theta )       ,  cos( theta)        , -L;
                     sin( theta - pi/3 ), -cos( theta - pi/3 ), -L ];
    end

    function b = cal_b()
        term1 = -dx * dtheta * cos(theta + pi/3) - dy * dtheta * sin(theta + pi/3);
        term2 =  dx * dtheta * cos(theta) + dy * dtheta * sin(theta);
        term3 = -dx * dtheta * cos(theta - pi/3) - dy * dtheta * sin(theta - pi/3);
        b = [term1; term2; term3];
    end

    function [M11, M12, M21, M22] = cal_M()
        M = diag([I1;
                  I1;
                  I1;
                  3*m1 + m2;
                  3*m1 + m2;
                  3*m1*L*L + I2;
                  ]);
        M11 = M(1:3,1:3);
        M22 = M(4:6,4:6);
        M12 = zeros(3,3);
        M21 = zeros(3,3);
    end

    function K = cal_K()
        K = A1^(-1) * A2;
    end

    function X = cal_X()
        X = K' * M11 * K + M22;
    end

    function [F1, F2] = cal_F()
        f1 = 0.75;  % 摩擦系数
        f2 = 1.25;  % 摩擦系数
        F1 = [-(atan(dphi1)/1.5708)*f1*(1/3)*9.8*(m1+3*m2)*r;
              -(atan(dphi2)/1.5708)*f1*(1/3)*9.8*(m1+3*m2)*r;
              -(atan(dphi3)/1.5708)*f1*(1/3)*9.8*(m1+3*m2)*r];
        F2 = [-(atan(dx)   /1.5708)*f2*(1/3)*9.8*(m1+3*m2)*(cos(pi/6 + theta) + cos(pi/6 - theta) + cos(pi/2 - theta));
              -(atan(dy)   /1.5708)*f2*(1/3)*9.8*(m1+3*m2)*(sin(pi/6 + theta) + sin(pi/6 - theta) + sin(pi/2 - theta));
              0];
    end
end
