import numpy as np
import util
import numdifftools as nd
import functools as ft

def get_point_qv_index(prob, t):
    qdim = prob['qdim']
    udim = prob['udim']
    pdim = 2*qdim + udim
    start = t*pdim
    end = start + 2*qdim
    index = np.array(range(start, end))
    return index


def get_point_index(prob, t):
    qdim = prob['qdim']
    udim = prob['udim']
    pdim = 2*qdim + udim
    start = t*pdim
    end = start + pdim
    index = np.array(range(start, end))
    return index


def get_dynamic_index(prob, t):
    t0_i = get_point_index(prob, t)
    t1_i = get_point_index(prob, t+1)
    return np.hstack([t0_i, t1_i])


def get_point_constriant(prob, t, goal, dims):
    index = get_point_qv_index(prob, t)
    eval_g = Goal_constriant(goal, dims)
    return index, eval_g


def get_dynamic_constriants(prob, dynamics, t_lst):
    index_lst = [get_dynamic_index(prob, t)
                 for t in t_lst]
    eval_g = Dynamics_constriant(prob, dynamics)
    return index_lst, eval_g


class Goal_constriant():
    def __init__(self, goal, dims=np.empty([])):
        self.goal = goal
        if dims.size==0 :
            dims = np.arange(self.goal.size)
        assert dims.size == self.goal.size
        self.dims = dims


    def __call__(self, x):
        error = x[self.dims] - self.goal
        error_2 = np.power(error, 2)
        return error_2



class Dynamics_constriant():
    def __init__(self, prob, dynamics):
        self.prob = prob
        self.dynamics = dynamics
        self.qdim = self.prob['qdim']
        self.udim = self.prob['udim']
        self.pdim = 2*self.qdim + self.udim


    def __call__(self, points):
        p0 = points[0:self.pdim]
        p1 = points[self.pdim:2*self.pdim]

        v0 = p0[self.qdim:2*self.qdim]
        v1 = p1[self.qdim:2*self.qdim]

        qv0 = p0[0:2*self.qdim]
        qv1 = p1[0:2*self.qdim]

        d0 = np.hstack([v0, self.dynamics(p0)])
        d1 = np.hstack([v1, self.dynamics(p1)])
        #3.2 of Kelly(2017)
        error = (qv1 - qv0) - 0.5*self.prob["dt"]*(d0 + d1)
        return error


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


class Eval_Lagrangian:
    def __init__(self, eval_f,  eval_g):
        self.eval_g = eval_g
        self.eval_f = eval_f

    def __call__(self, x, lagrange, obj_factor, user_data = None):
        return obj_factor * self.eval_f(x) + np.dot(lagrange, self.eval_g(x))





