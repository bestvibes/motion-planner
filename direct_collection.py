import numpy as np
import scipy as sp
import itertools as it
import pyipopt as ip
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


def block_dymamics(q, v, u):
    return u


def block_dymamics_jac(q, v, u):
    jac = np.hstack([np.zeros_like(q), np.zeros_like(v), np.ones_like(u)])
    jac_2d = jac.reshape(1, -1)
    return jac_2d


class Point_constraint():
    def __init__(self, goal, t, qdim, udim, dt):
        self.goal = goal
        self.t = t
        self.qdim = qdim
        self.udim = udim
        self.dt = dt


class Point_Dynamic_Error():
    def __init__(self, problem, dynamics, t):
        self.prob = problem
        self.dynamics = dynamics
        self.t = t

    def __call__(self, traj):
        #return a vector, representing the dynamic error at time i
        #the dimension of the vector is ||2 * qdim|| for q and v
        q0, v0, u0 = get_point_q_v_u(traj, self.t, prob["qdim"], prob["udim"])
        q1, v1, u1 = get_point_q_v_u(traj, self.t+1, prob["qdim"], prob["udim"])

        x0 = np.hstack([q0, v0])
        x1 = np.hstack([q1, v1])

        # the deviavie respect to [q, v], not including u: x_dot = f(x, u)
        #TODO: each point is evaluted twice; this can be improved
        d0 = np.hstack([v0, self.dynamics(q0, v0, u0)])
        d1 = np.hstack([v1, self.dynamics(q1, v1, u1)])
        error = (x1 - x0) - 0.5*self.prob["dt"]*(d0 + d1) #3.2 of Kelly(2017)
        return error


class Point_Dynamic_Jacobian():
    def __init__(self, problem, dynamics_jac, t):
        self.t = t
        self.prob = problem
        # dynamics_jac evaluats the jacobian of accelation = f(q, v, u)
        self.dynamics_jac = dynamics_jac
        #the jacobian of the positon error is determined by postion and  velocity
        #And it is not influenced by the dynamic_jac, which is associated with acc
        #This is a constant so can be set before run time
        self.dim_q_jac_lst = [-1, -0.5*self.prob["dt"], 1, -0.5*self.prob["dt"]]
        self.q_jac_lst = self.dim_q_jac_lst * self.prob['qdim']

        #the position of non-zero jacobian with shape(2*qdim, ||traj||)
        self.position = self.get_position()


    def get_position(self):
        dq_position = self.get_jq_position()
        dv_position = self.get_jv_position()
        position = dq_position + dv_position
        return position


    def get_jq_position(self):
        pdim = 2 * self.prob["qdim"] + self.prob["udim"]
        qdim = self.prob["qdim"]
        t = self.t
        def get_dim_jac_position(t, d):
            q0_i = pdim*t + d
            q1_i = pdim*(t+1) + d
            v0_i = q0_i + qdim
            v1_i = q1_i + qdim
            return [q0_i, v0_i, q1_i, v1_i]

        q_position_lst = [get_dim_jac_position(t, d) for d in range(qdim)]
        return q_position_lst

    def get_jv_position(self):
        qdim = self.prob["qdim"]
        udim = self.prob["udim"]
        t = self.t
        q0_index, v0_index, u0_index = get_point_index(t, qdim, udim)
        q1_index, v1_index, u1_index = get_point_index(t+1, qdim, udim)
        row_v_position = list(range(q0_index[0], u1_index[-1]))#TODO: check end point
        # v_position = row_v_position*qdim
        v_position = [row_v_position for d in range(qdim)]
        return v_position


    def value(self, traj):
        #TODO: check this function !!
        #values of non-zero jacobian; the full jacobian is (2*qdim, ||traj||)
        #the jacobian of the positon error is determined by postion and  velocity
        #And it is not influenced by the dynamic_jac, which is associated with acc
        # q_jac_lst = [-1, -0.5*self.prob["dt"], 1, -0.5*self.prob["dt"]] * self.qdim
        # postion derivative
        t = self.t
        dt = self.prob["dt"]
        qdim = self.prob["qdim"]
        udim = self.prob["udim"]

        q0, v0, u0 = get_point_q_v_u(traj, t, qdim, udim)
        q1, v1, u1 = get_point_q_v_u(traj, t+1, qdim, udim)

        a0_jac = self.dynamics_jac(q0, v0, u0)
        a1_jac = self.dynamics_jac(q1, v1, u1)

        dq0 = -0.5*dt*a0_jac[:,0:qdim]
        dv0 = -1 - 0.5*dt*a0_jac[:, qdim:2*qdim]
        du0 = -0.5*dt*a0_jac[:, 2*qdim:2*qdim+udim]

        dq1 = -0.5*dt*a1_jac[:,0:qdim]
        dv1 = 1 - 0.5*dt*a1_jac[:, qdim:2*qdim]
        du1 = -0.5*dt*a1_jac[:, 2*qdim:2*qdim+udim]

        v_jac = np.concatenate([dq0, dv0, du0, dq1, dv1, du1], axis=1)
        v_jac_flat = v_jac.flatten()

        point_jac_flat = np.hstack([self.q_jac_lst, v_jac_flat])
        return point_jac_flat


    def __call__(self, traj, flag):
        if flag:
            return self.position
        return self.value(traj)


class Point_Dynamic_Hessian:
    def __init__(self, problem, dynamic_hessian, t):
        self.prob = prob
        self.dynamic_hessian = dynamic_hessian
        self.t = t
        self.position = self.get_position()

    def get_position(self):
        pass


    def value(self, X):
        pass

    def __call__(traj, flag):
        if flag:
            return self.position
        value = self.value(traj)
        return value



def traj_dynamic_error_function_factory(prob, dynamics):
    #number of trajectory point
    n = prob["n"]
    #there is no dynamic constriant for the last point
    func_lst = [Point_Dynamic_Error(prob, dynamics, t) for t in range(n-1)]
    return func_lst


def traj_dynamic_jacobian_function_factory(prob, dynamics_jac):
    n = prob["n"]
    #there is no dynamic constriant for the last point
    func_lst =[Point_Dynamic_Jacobian(prob, dynamics_jac, t)
               for t in range(n-1)]
    return func_lst


class Ipopt_Constriants:
    def __init__(self, g_func_lst):
        self.g_func_lst = g_func_lst

    def __call__(self, X):
        error_lst = [g(X) for g in self.g_func_lst]
        error_arr = np.hstack(error_lst)
        return error_arr


class Ipopt_Constriants_Jacobian:
    def __init__(self, g_jac_func_lst):
        self.g_jac_func_lst = g_jac_func_lst
        self.N = len(self.g_jac_func_lst)

    def __call__(self, X, flag):
        result_iter = (g_jac(X, flag) for g_jac in self.g_jac_func_lst)
        if flag:# return positions, not values
            col_pos_iter = result_iter
            row_col_iter = it.chain.from_iterable(col_pos_iter)
            pos_lst = [[(r, c) for c in cs] for (r, cs) in enumerate(row_col_iter)]
            pos_arr = np.vstack(pos_lst)
            row_arr = pos_arr[:, 0]
            col_arr = pos_arr[:, 1]
            return row_arr, col_arr
        value_iter = result_iter
        value_lst = list(value_iter)
        value_arr = np.hstack(value_lst)
        return value_arr


if __name__=="__main__":
    prob = {}
    prob["n"] = 20 # number of trajectory point
    prob["dt"] = 1.0/prob["n"]
    prob["start"] = (0, 0)
    prob["end"] = (1, 1)
    prob["qdim"] = 1
    prob["udim"] = 1


    start = prob["start"]
    end = prob["end"]
    n = prob['n']
    q_arr = np.linspace(start[0], end[0], n)
    v_arr = np.linspace(start[1], end[1], n)
    u_arr = np.zeros_like(q_arr, dtype=float)
    X_init = np.vstack([q_arr, v_arr, u_arr]).flatten("F")

    dynamic_g_lst = traj_dynamic_error_function_factory(prob, block_dymamics)
    dynamic_g_jac_lst = traj_dynamic_jacobian_function_factory(prob, block_dymamics_jac)
    ipopt_g = Ipopt_Constriants(dynamic_g_lst)
    ipopt_g_jac = Ipopt_Constriants_Jacobian(dynamic_g_jac_lst)

    jac_pos = ipopt_g_jac(X_init, True)
    jac_value = ipopt_g_jac(X_init, False)




