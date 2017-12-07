import pyipopt
import numpy as np
import math
import itertools as it

def get_point_index(t, qdim, udim):
    # qdim, udim = self.qdim, self.udim
    point_dim = 2*qdim + udim
    start = t*point_dim
    qt_index = [start, start+qdim]
    vt_index = [qt_index[-1], qt_index[-1]+qdim]
    ut_index = [vt_index[-1], vt_index[-1]+udim]
    return qt_index, vt_index, ut_index

def get_point_q_v_u(traj, t, qdim, udim):
    q_index, v_index, u_index = get_point_index(t, qdim, udim)
    q = traj[q_index[0]:q_index[-1]]
    v = traj[v_index[0]:v_index[-1]]
    u = traj[u_index[0]:u_index[-1]]
    return q, v, u

def eval_f(X):
    u = X[-1]
    return math.pow(u, 2)

class Control_Cost():
    def __init__(self, prob):
        self.prob = prob

    def eval_f(self, X):
        n = prob["n"]
        qdim = prob["qdim"]
        udim = prob["udim"]
        x_2d = X.reshape(n, 2*qdim+udim)
        u = x_2d[:, 2*qdim:]
        value = np.sum(np.power(u, 2))
        return value


    def eval_grad_f(self, X):
        n = prob["n"]
        qdim = prob["qdim"]
        udim = prob["udim"]
        x_2d = X.reshape(n, 2*qdim+udim)
        u = x_2d[:, 2*qdim:]

        grad_2d = np.zeros_like(x_2d)
        grad_2d[:, 2*qdim:] = u
        grad = grad_2d.flatten()
        return grad


class Goal_constriant():
    def __init__(self, prob, goal, t):
        self.prob = prob
        self.goal = goal
        self.t = t

    def eval_g(self, X):
        q, v, u = get_point_q_v_u(X, self.t, self.prob['qdim'], self.prob['udim'])
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
        q, v, u = get_point_q_v_u(X, self.t, qdim, udim)
        error = np.hstack([q, v]) - self.goal
        jac_value = 2*error
        return jac_value


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


if __name__=="__main__":

    X = np.array([0, 0, 10, 0, 0, 2, 0, 0, 5])
    goal1 = np.array([2, 3])
    goal2 = np.array([4, 5])

    prob = {}
    prob["n"] = 3
    prob["qdim"] = 1
    prob["udim"] = 1

    cost = Control_Cost(prob)
    foo = cost.eval_f(X)

    c1 = Goal_constriant(prob, goal1, 0)
    c1_g = c1.eval_g
    c1_jac_g = c1.eval_jac_g

    c2 = Goal_constriant(prob, goal2, 1)
    c2_g = c2.eval_g
    c2_jac_g = c2.eval_jac_g

    eval_g_lst = [c1_g, c2_g]
    eval_jac_g_lst = [c1_jac_g, c2_jac_g]

    eval_g = Ipopt_Constriants(eval_g_lst)
    eval_jac_g = Ipopt_Constriants_Jacobian(eval_g_lst, eval_jac_g_lst)


    g = eval_g(X)
    jac = eval_jac_g(X, False)
    mask = eval_jac_g(X, True)

    x_L = np.ones(X.size)*-100
    x_U = np.ones(X.size)*100


    nvar = X.size
    ncon = eval_g.get_num_constraints(X)

    g_L = np.zeros(ncon)
    g_U = np.zeros(ncon)

    nnzj = eval_jac_g.get_nnz(X)

    nlp = pyipopt.create(nvar, x_L, x_U, ncon, g_L, g_U, nnzj, 0, cost.eval_f,
                        cost.eval_grad_f, eval_g, eval_jac_g)


    output, zl, zu, constraint_multipliers, obj, status = nlp.solve(X)

    print (output)

