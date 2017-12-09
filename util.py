import numpy as np
import numdifftools as nd

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

