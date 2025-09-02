#!/usr/bin/env python
import numpy as np
import casadi as ca     # casadi求解器，下面MPC的非线性优化问题将使用casadi求解
import time


class MPC(object):
    def __init__(self) -> None:
        self.DIFF_CAR  = 2
        self.OMNI_CAR  = 3

    def set_type(self, car_type):
        """ 设置车辆类型 """
        self.type      = car_type

    def set_time(self, T, step_len):
        """ 设置和时间相关的变量 """
        self.T         = T          # 离散时间间隔
        self.N         = step_len   # 预测步长

    def set_max_speed(self, vx_min, vx_max, vy_min, vy_max, omega_min, omega_max):
        """ 设置最高运行速度 """
        self.speed_demand = (vx_min, vx_max, vy_min, vy_max, omega_min, omega_max)

    def set_QR(self, Q, R):
        """ 设置QR矩阵 """
        self.Q = Q
        self.R = R

    def set_opti(self):
        """ 构建优化问题 """
        self.opti      = ca.Opti()
        self.goal      = self.opti.parameter(3, self.N)
        self.goalx     = self.goal[0, :]
        self.goaly     = self.goal[1, :]
        self.goaltheta = self.goal[2, :]
        # 控制变量：速度v，角速度omega
        self.opt_controls     = self.opti.variable(3, self.N)
        self.vx               = self.opt_controls[0, :]
        self.vy               = self.opt_controls[1, :]
        self.omega            = self.opt_controls[2, :]
        # 状态变量：x，y，theta
        self.opt_states       = self.opti.variable(3, self.N+1)
        self.x                = self.opt_states[0, :]
        self.y                = self.opt_states[1, :]
        self.theta            = self.opt_states[2, :]
        # 速度要求
        vx_min, vx_max, vy_min, vy_max, omega_min, omega_max = self.speed_demand
        self.opti.subject_to(self.opti.bounded(vx_min, self.vx, vx_max))
        self.opti.subject_to(self.opti.bounded(vy_min, self.vy, vy_max))
        self.opti.subject_to(self.opti.bounded(omega_min, self.omega, omega_max))
        # 运动学约束
        self._set_model_and_cost_function()


    def _set_model_and_cost_function(self):
        # 运动学方程
        if self.type == self.DIFF_CAR:
            f = lambda x_, u_, T_: T_*ca.vertcat(*[u_[0,:]*ca.cos(x_[2,:]), u_[0,:]*ca.sin(x_[2,:]), u_[2,:]])
        # 运动学约束
        X_this_matrix = self.opt_states[:,:-1]
        X_next_matrix = self.opt_states[:,1:]
        self.opti.subject_to( X_next_matrix == X_this_matrix + f(X_this_matrix, self.opt_controls, self.T) )
        # 代价函数
        ex        = self.x[:,:-1] - self.goalx
        ey        = self.y[:,:-1] - self.goaly
        etheta    = self.theta[:,:-1] - self.goaltheta
        self.cost = 0.8*self.Q[0,0]*ca.mtimes(ex,      ex.T)      + self.Q[1,1]*ca.mtimes(ey,      ey.T)      + self.Q[2,2]*ca.mtimes(etheta,     etheta.T) + \
                    0.5*self.R[0,0]*ca.mtimes(self.vx, self.vx.T) + self.R[1,1]*ca.mtimes(self.vy, self.vy.T) + self.R[2,2]*ca.mtimes(self.omega, self.omega.T)


    def set_init(self, init_state=[]):
        """ 设置起点约束 """
        self.opti.subject_to(self.opt_states[0,0] == init_state[0])
        self.opti.subject_to(self.opt_states[1,0] == init_state[1])
        self.opti.subject_to(self.opt_states[2,0] == init_state[2])

    def set_goal(self, goal):
        """ 设置终点 """
        self.opti.set_value(self.goal, np.array(goal).T)


    def run(self):
        """ 求解 """
        self.opti.minimize(self.cost)
        opts_setting = {'ipopt.max_iter':100, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-4, 'ipopt.acceptable_obj_change_tol':1e-4}
        self.opti.solver('ipopt', opts_setting)

        try:
            sol       = self.opti.solve()                        # 优化!!!!!!!!!!!!!
            u_res     = sol.value(self.opt_controls)             # 获得最优控制输入序列
            state_res = sol.value(self.opt_states)               # 获得最优状态序列
        except:
            u_res     = [[0, 0, 0]] * int(self.T)
            state_res = self.opti.debug.value(self.opt_states)   # 获得最优状态序列

        return state_res, u_res

