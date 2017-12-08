import numpy as np
import util

class Goal_constriant():
    def __init__(self, prob, goal, t):
        self.prob = prob
        self.goal = goal
        self.t = t

    def eval_g(self, X):
        q, v, u = util.get_point_q_v_u(X, self.t, self.prob['qdim'], self.prob['udim'])
        error = np.hstack([q, v]) - self.goal
        error_2 = np.power(error, 2)
        return error_2

    def eval_jac_g(self, X, flag):
        qdim = self.prob["qdim"]
        udim = self.prob["udim"]
        if flag:
            pdim = 2*qdim + udim
            start = pdim*self.t
            end = start + 2*qdim
            row = np.arange(0, 2*qdim)
            col = np.arange(start, end)
            return (row, col)
        q, v, u = util.get_point_q_v_u(X, self.t, qdim, udim)
        error = np.hstack([q, v]) - self.goal
        jac_value = 2*error
        return jac_value


class Dynamics_constriant():
    def __init__(self, prob, dynamics, dynamics_jac, t):
        #the size of the eval_g is 2*qdim, for q_dot and q_dot_dot
        #the shape of the jacobian is ( 2*qdim,  ||traj|| )
        self.prob = prob
        self.dynamics = dynamics
        self.dynamics_jac = dynamics_jac
        self.t = t
        self.get_indexes(t)

        #the jacobian of the positon error is determined by postion and  velocity
        #And it is not influenced by the dynamic_jac, which is associated with acc
        #This is a constant so can be set before run time
        self.vel_dim_jac = [-1, -0.5*self.prob["dt"], 1, -0.5*self.prob["dt"]]
        self.vel_jac_lst = self.vel_dim_jac * self.prob['qdim']

    def eval_g(self, traj):
        q0, v0, u0 = util.get_q_v_u_from_indexes(traj, self.q0_i, self.v0_i, self.u0_i)
        q1, v1, u1 = util.get_q_v_u_from_indexes(traj, self.q1_i, self.v1_i, self.u1_i)

        x0 = np.hstack([q0, v0])
        x1 = np.hstack([q1, v1])

        d0 = np.hstack([v0, self.dynamics(q0, v0, u0)])
        d1 = np.hstack([v1, self.dynamics(q1, v1, u1)])
        #3.2 of Kelly(2017)
        error = (x1 - x0) - 0.5*self.prob["dt"]*(d0 + d1)
        return error

    def get_indexes(self, t):
        qdim = self.prob["qdim"]
        udim = self.prob["udim"]
        self.q0_i, self.v0_i, self.u0_i = util.get_point_index(t, qdim, udim)
        self.q1_i, self.v1_i, self.u1_i = util.get_point_index(t+1, qdim, udim)


    def get_constant_vel_jac_value(self):
        self.dim_q_jac_lst = [-1, -0.5*self.prob["dt"], 1, -0.5*self.prob["dt"]]
        #repeat this with qdim times, one for each qdim
        self.q_jac_lst = self.dim_q_jac_lst * self.prob['qdim']


    def get_vel_jac_positions(self):
        qdim = self.prob["qdim"]
        udim = self.prob["udim"]
        pdim = 2 * qdim + udim
        t = self.t
        def get_dim_jac_cols(t, d):
            q0_i = pdim*t + d
            q1_i = pdim*(t+1) + d
            v0_i = q0_i + qdim
            v1_i = q1_i + qdim
            return [q0_i, v0_i, q1_i, v1_i]

        cols_lst = [get_dim_jac_cols(t, d) for d in range(qdim)]
        rows_lst = [[r]*len(cols) for (r, cols) in enumerate(cols_lst)]

        cols_arr = np.hstack(cols_lst)
        rows_arr = np.hstack(rows_lst)
        return rows_arr, cols_arr

    def get_acc_jac_positions(self):
        qdim = self.prob["qdim"]
        cols_lst = [list(range(self.q0_i[0], self.u1_i[-1])) for i in range(qdim)]
        #acc starts from row qdim
        rows_lst = [[r+qdim]*len(cols) for (r, cols) in enumerate(cols_lst)]

        cols_arr = np.hstack(cols_lst)
        rows_arr = np.hstack(rows_lst)
        return rows_arr, cols_arr

    def eval_jac_g(self, traj, flag):
        if flag:
            vel_rows, vel_cols = self.get_vel_jac_positions()
            acc_rows, acc_cols = self.get_acc_jac_positions()
            rows = np.hstack([vel_rows, acc_rows])
            cols = np.hstack([vel_cols, acc_cols])
            return rows, cols

        dt = self.prob["dt"]
        qdim = self.prob["qdim"]
        udim = self.prob["udim"]

        q0, v0, u0 = util.get_q_v_u_from_indexes(traj, self.q0_i, self.v0_i, self.u0_i)
        q1, v1, u1 = util.get_q_v_u_from_indexes(traj, self.q1_i, self.v1_i, self.u1_i)

        a0_jac = self.dynamics_jac(q0, v0, u0)
        a1_jac = self.dynamics_jac(q1, v1, u1)

        dq0 = -0.5*dt*a0_jac[:,0:qdim]
        dv0 = -1 - 0.5*dt*a0_jac[:, qdim:2*qdim]
        du0 = -0.5*dt*a0_jac[:, 2*qdim:2*qdim+udim]

        dq1 = -0.5*dt*a1_jac[:,0:qdim]
        dv1 = 1 - 0.5*dt*a1_jac[:, qdim:2*qdim]
        du1 = -0.5*dt*a1_jac[:, 2*qdim:2*qdim+udim]

        acc_jac = np.concatenate([dq0, dv0, du0, dq1, dv1, du1], axis=1)
        acc_jac_flat = acc_jac.flatten()

        jac_flat = np.hstack([self.vel_jac_lst, acc_jac_flat])
        return jac_flat


class Ipopt_Constriants:
    def __init__(self, g_func_lst):
        self.g_func_lst = g_func_lst

    def get_num_constraints(self, X):
        error_arr = self.__call__(X)
        return error_arr.size

    def __call__(self, X):
        error_lst = [g(X) for g in self.g_func_lst]
        error_arr = np.hstack(error_lst)
        return error_arr


class Ipopt_Constriants_Jacobian:
    def __init__(self, g_func_lst, g_jac_func_lst):
        assert len(g_func_lst) == len(g_jac_func_lst)
        self.g_func_lst = g_func_lst
        self.g_jac_func_lst = g_jac_func_lst
        self.N = len(self.g_jac_func_lst)

    def get_start_row_lst(self, X):
        size_lst = [0] + [g(X).size for g in self.g_func_lst[0:-1]]
        start_row_lst = list( np.cumsum(size_lst))
        return start_row_lst

    def get_nnz(self, X):
        value_arr = self.__call__(X, False)
        return value_arr.size


    def __call__(self, X, flag):
        result_iter = (g_jac(X, flag) for g_jac in self.g_jac_func_lst)
        if flag:# return positions, not values
            start_row_lst = self.get_start_row_lst(X)
            rows_cols_iter = result_iter
            raw_rows_lst, cols_lst = zip(*result_iter)
            cols = np.hstack(cols_lst)
            row_lst =[raw_rows+start_row_lst[i]
                      for (i, raw_rows) in enumerate(raw_rows_lst)]
            rows = np.hstack(row_lst)
            return rows, cols
        value_iter = result_iter
        value_lst = list(value_iter)
        value_arr = np.hstack(value_lst)
        return value_arr


