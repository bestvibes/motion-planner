import numpy as np
import util
import numdifftools as nd
import functools as ft
import adolc

class Goal_constriant():
    def __init__(self, prob, goal, t):
        self.prob = prob
        self.goal = goal
        self.t = t


    def get_indexes(self):
        qdim = self.prob['qdim']
        udim = self.prob['udim']
        pdim = 2*qdim + udim
        start = self.t*pdim
        end = start + 2*qdim
        indexes = np.array(range(start, end))
        return indexes

    def eval_g(self, qv):
        # q, v, u = util.get_point_q_v_u(X, self.t, self.prob['qdim'], self.prob['udim'])
        error = np.hstack(qv) - self.goal
        error_2 = np.power(error, 2)
        return error_2

    # def eval_jac_g(self, X, flag):
    #     qdim = self.prob["qdim"]
    #     udim = self.prob["udim"]
    #     if flag:
    #         pdim = 2*qdim + udim
    #         start = pdim*self.t
    #         end = start + 2*qdim
    #         row = np.arange(0, 2*qdim)
    #         col = np.arange(start, end)
    #         return (row, col)
    #     q, v, u = util.get_point_q_v_u(X, self.t, qdim, udim)
    #     error = np.hstack([q, v]) - self.goal
    #     jac_value = 2*error
    #     return jac_value


class Dynamics_constriant():
    def __init__(self, prob, dynamics,  t):
        #the size of the eval_g is 2*qdim, for q_dot and q_dot_dot
        #the shape of the jacobian is ( 2*qdim,  ||traj|| )
        self.prob = prob
        self.dynamics = dynamics
        # self.dynamics_jac = dynamics_jac
        self.t = t
        # self.get_indexes(t)

        #the jacobian of the positon error is determined by postion and  velocity
        #And it is not influenced by the dynamic_jac, which is associated with acc
        #This is a constant so can be set before run time
        self.vel_dim_jac = [-1, -0.5*self.prob["dt"], 1, -0.5*self.prob["dt"]]
        self.vel_jac_lst = self.vel_dim_jac * self.prob['qdim']

    def eval_g(self, points):
        qdim = self.prob['qdim']
        udim = self.prob['udim']
        pdim = 2*qdim + udim
        p0 = points[0:pdim]
        p1 = points[pdim:2*pdim]

        v0 = p0[qdim:2*qdim]
        v1 = p1[qdim:2*qdim]

        qv0 = p0[0:2*qdim]
        qv1 = p1[0:2*qdim]

        #TODO: p0 can be changed to a tuple so that dynamics can be memorized
        d0 = np.hstack([v0, self.dynamics(p0)])
        d1 = np.hstack([v1, self.dynamics(p1)])
        #3.2 of Kelly(2017)
        error = (qv1 - qv0) - 0.5*self.prob["dt"]*(d0 + d1)
        return error

    # def get_indexes(self, t):
    #     qdim = self.prob["qdim"]
    #     udim = self.prob["udim"]
    #     self.q0_i, self.v0_i, self.u0_i = util.get_point_index(t, qdim, udim)
    #     self.q1_i, self.v1_i, self.u1_i = util.get_point_index(t+1, qdim, udim)

    def get_indexes(self):
        qdim = self.prob["qdim"]
        udim = self.prob["udim"]
        pdim = 2*qdim + udim
        start = self.t*pdim
        end = start+2*pdim
        # q0_i, v0_i, u0_i = util.get_point_index(self.t, qdim, udim)
        # q1_i, v1_i, u1_i = util.get_point_index(t+1, qdim, udim)
        indexes = np.array(range(start, end))
        return indexes

    # def get_constant_vel_jac_value(self):
    #     self.dim_q_jac_lst = [-1, -0.5*self.prob["dt"], 1, -0.5*self.prob["dt"]]
    #     #repeat this with qdim times, one for each qdim
    #     self.q_jac_lst = self.dim_q_jac_lst * self.prob['qdim']
    #
    #
    # def get_vel_jac_positions(self):
    #     qdim = self.prob["qdim"]
    #     udim = self.prob["udim"]
    #     pdim = 2 * qdim + udim
    #     t = self.t
    #     def get_dim_jac_cols(t, d):
    #         q0_i = pdim*t + d
    #         q1_i = pdim*(t+1) + d
    #         v0_i = q0_i + qdim
    #         v1_i = q1_i + qdim
    #         return [q0_i, v0_i, q1_i, v1_i]
    #
    #     cols_lst = [get_dim_jac_cols(t, d) for d in range(qdim)]
    #     rows_lst = [[r]*len(cols) for (r, cols) in enumerate(cols_lst)]
    #
    #     cols_arr = np.hstack(cols_lst)
    #     rows_arr = np.hstack(rows_lst)
    #     return rows_arr, cols_arr
    #
    # def get_acc_jac_positions(self):
    #     qdim = self.prob["qdim"]
    #     cols_lst = [list(range(self.q0_i[0], self.u1_i[-1])) for i in range(qdim)]
    #     #acc starts from row qdim
    #     rows_lst = [[r+qdim]*len(cols) for (r, cols) in enumerate(cols_lst)]
    #
    #     cols_arr = np.hstack(cols_lst)
    #     rows_arr = np.hstack(rows_lst)
    #     return rows_arr, cols_arr
    #
    # def eval_jac_g(self, traj, flag):
    #     if flag:
    #         vel_rows, vel_cols = self.get_vel_jac_positions()
    #         acc_rows, acc_cols = self.get_acc_jac_positions()
    #         rows = np.hstack([vel_rows, acc_rows])
    #         cols = np.hstack([vel_cols, acc_cols])
    #         return rows, cols
    #
    #     dt = self.prob["dt"]
    #     qdim = self.prob["qdim"]
    #     udim = self.prob["udim"]
    #
    #     # q0, v0, u0 = util.get_q_v_u_from_indexes(traj, self.q0_i, self.v0_i, self.u0_i)
    #     # q1, v1, u1 = util.get_q_v_u_from_indexes(traj, self.q1_i, self.v1_i, self.u1_i)
    #     qvu0 = traj[self.q0_i[0]:self.u0_i[-1]]
    #     qvu1 = traj[self.q0_i[0]:self.u0_i[-1]]
    #
    #     a0_jac = self.dynamics_jac(qvu0)
    #     a1_jac = self.dynamics_jac(qvu1)
    #
    #     dq0 = -0.5*dt*a0_jac[:,0:qdim]
    #     dv0 = -1 - 0.5*dt*a0_jac[:, qdim:2*qdim]
    #     du0 = -0.5*dt*a0_jac[:, 2*qdim:2*qdim+udim]
    #
    #     dq1 = -0.5*dt*a1_jac[:,0:qdim]
    #     dv1 = 1 - 0.5*dt*a1_jac[:, qdim:2*qdim]
    #     du1 = -0.5*dt*a1_jac[:, 2*qdim:2*qdim+udim]
    #
    #     acc_jac = np.concatenate([dq0, dv0, du0, dq1, dv1, du1], axis=1)
    #     acc_jac_flat = acc_jac.flatten()
    #
    #     jac_flat = np.hstack([self.vel_jac_lst, acc_jac_flat])
    #     return jac_flat


class Stacked_Constriants:
    def __init__(self, g_func_lst, indexes_lst):
        # self.g_func_lst = g_func_lst
        self.g_i_lst = list(zip(g_func_lst, indexes_lst))

    def get_num_constraints(self, X):
        error_arr = self.__call__(X)
        return error_arr.size

    def __call__(self, X):
        error_lst = [g(X[i]) for (g, i) in self.g_i_lst]
        error_arr = np.hstack(error_lst)
        return error_arr


class Stacked_Constriants_Jacobian:
    def __init__(self, g_func_lst, g_jac_func_lst, indexes_lst):
        assert len(g_func_lst) == len(g_jac_func_lst) == len(indexes_lst)
        self.g_func_lst = g_func_lst
        self.g_jac_func_lst = g_jac_func_lst
        self.indexs_lst = indexes_lst
        # self.N = len(self.g_jac_func_lst)
        self.g_gjac_i_lst = list(zip(g_func_lst, g_jac_func_lst, indexes_lst))

    def get_start_row_lst(self, X):

        size_lst = [0] + [g(X[i]).size for (g, _, i) in self.g_gjac_i_lst[0:-1]]
        start_row_lst = list( np.cumsum(size_lst))
        return start_row_lst

    def get_nnz(self, X):
        value_arr = self.__call__(X, False)
        return value_arr.size


    def __call__(self, X, flag):
        # result_iter = (g_jac(X, flag) for g_jac in self.g_jac_func_lst)
        result_iter = (g_jac(X[i], flag) for (_, g_jac, i) in self.g_gjac_i_lst)
        if flag:# return positions, not values
            start_row_lst = self.get_start_row_lst(X)
            local_cols_iter = result_iter
            local_rows_lst, local_cols_lst = zip(*result_iter)
            #convering local cols into global cols
            cols_lst = [self.indexs_lst[i][c]
                        for (i, c) in enumerate(local_cols_lst)]

            cols = np.hstack(cols_lst)
            row_lst =[local_rows+start_row_lst[i]
                      for (i, local_rows) in enumerate(local_rows_lst)]
            rows = np.hstack(row_lst)
            return rows, cols
        value_iter = result_iter
        value_lst = list(value_iter)
        value_arr = np.hstack(value_lst)
        return value_arr


class Sparse_Jacobian:
    def __init__(self, eval_g, x_L, x_U):
        self.J_func = nd.Jacobian(eval_g)
        assert x_L.size >0 and x_L.size == x_U.size
        self.rows, self.cols = util.get_func_sparse_pattern(eval_g, x_L, x_U)


    def __call__(self, X, flag):
        if flag:
            return self.rows, self.cols

        J = self.J_func(X)
        values = J[self.rows, self.cols]
        return values

class Sparse_Jacobian_Adolc:
    def __init__(self, eval_g, x_L, x_U, i):
        self.i = i
        # self.J_func = nd.Jacobian(eval_g)
        assert x_L.size >0 and x_L.size == x_U.size
        # self.rows, self.cols = util.get_func_sparse_pattern(eval_g, x_L, x_U)
        x = np.random.uniform(x_L, x_U)
        adolc.trace_on(i)
        ax = adolc.adouble(x)
        adolc.independent(ax)
        ay = eval_g(ax)
        # y = get_constriant_func(eval_g(x))
        adolc.dependent(ay)
        adolc.trace_off()
        self.options = np.array([0, 0, 0, 0], dtype=int)
        jac_arr = adolc.jacobian(i, x)
        self.rows, self.cols = np.where(jac_arr!=0)

    def __call__(self, X, flag):
        if flag:
            return self.rows, self.cols
        result = adolc.colpack.sparse_jac_no_repeat(self.i, X, self.options)
        values = result[-1]
        return values

        # J = self.J_func(X)
        # values = J[self.rows, self.cols]
        # return values

class Sparse_Jacobian_offline:
    def __init__(self, eval_g, rows, cols):
        self.J_func = nd.Jacobian(eval_g)
        self.rows = rows
        self.cols = cols

    def __call__(self, X, flag):
        if flag:
            return self.rows, self.cols

        J = self.J_func(X)
        values = J[self.rows, self.cols]
        return values
