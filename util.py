import numpy as np
import numdifftools as nd
import functools as ft

def get_point_index(t, qdim, udim):
    # qdim, udim = self.qdim, self.udim
    point_dim = 2*qdim + udim
    start = t*point_dim
    qt_index = [start, start+qdim]
    vt_index = [qt_index[-1], qt_index[-1]+qdim]
    ut_index = [vt_index[-1], vt_index[-1]+udim]
    return qt_index, vt_index, ut_index


def get_q_v_u_from_indexes(traj, q_index, v_index, u_index):
    q = traj[q_index[0]:q_index[-1]]
    v = traj[v_index[0]:v_index[-1]]
    u = traj[u_index[0]:u_index[-1]]
    return q, v, u


def get_point_q_v_u(traj, t, qdim, udim):
    q_index, v_index, u_index = get_point_index(t, qdim, udim)
    return get_q_v_u_from_indexes(traj, q_index, v_index, u_index)


def get_func_sparse_pattern(func, x_L, x_U, n=3):
    assert x_L.size == x_U.size
    J_func = nd.Jacobian(func)
    X_arr = np.random.uniform(x_L, x_U, (n, x_L.size))
    J_lst = [J_func(X) for X in X_arr]
    mask_lst = [J!=0 for J in J_lst]
    mask = np.logical_or.reduce(mask_lst)
    rows, cols = np.where(mask)
    # for some reason, only the copied ones work !
    J_rows = np.copy(rows)
    J_cols = np.copy(cols)
    return J_rows, J_cols


class memorize_func():
    def __init__(self, vec_func):
        self._vec_func = vec_func


    @ft.lru_cache(maxsize=200)
    def __call__(self, x_t):
        x_arr = np.asarray(x_t)
        result = self._vec_func(x_arr)
        return result



class scaler_output(object):

    """returning a single dimension of a function's output"""

    def __init__(self, vec_func, i):
        """returning the ith dimension of vec_funct's output

        :vec_func: a function with vector outputs
        :i: the ith dimension of the outputs to return
        """
        self._vec_func = vec_func
        self._i = i


    def __call__(self, X):
        x_t = tuple(X)
        out_vec = self._vec_func(x_t)
        return out_vec[self._i]



def vec_func_to_scaler_func_lst(vec_func, outdim):
    """splitting a function with a vector output to a list of funcs with scaler
    output.

    :vec_func: a function with vector input and vector outputs
    :outdim: the dimension of the output
    :returns: [function with scaler output]

    """
    memorized  = memorize_func(vec_func)
    scaler_func_lst = [scaler_output(memorized, i) for i in range(outdim)]
    return scaler_func_lst

class Hessian_Lst(object):

    """given an vector output func, returing a list of hessian for an X"""

    def __init__(self, vec_func, outdim):
        """TODO: to be defined1.

        :vec_func: TODO
        :outdim: the dimension of vec_func's output

        """
        self._vec_func = vec_func
        self._outdim = outdim
        self._scaler_func_lst = vec_func_to_scaler_func_lst(vec_func, outdim)
        self._H_func_lst = [nd.Hessian(f) for f in self._scaler_func_lst]

    def __call__(self, X):
        H_lst = [H_func(X) for H_func in self._H_func_lst]
        return H_lst


def test_hessian():

    ndim = 100
    f = lambda x: np.power(x, 2)
    h_lst_func = Hessian_Lst(f, ndim)

    x_L = np.ones(ndim)*(-5)
    x_U = np.ones(ndim)*(5)

    np.random.seed(42)
    x = np.random.uniform(x_L, x_U, ndim)

    h_lst = h_lst_func(x)

    print (h_lst[0])


if __name__ == "__main__":
    test_hessian()



