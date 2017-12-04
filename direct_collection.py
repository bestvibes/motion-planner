import numpy as np
import scipy as sp
import itertools as it
import pyipopt as ip

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
    def __init__(self, index, problem, dynamics):
        self.t = index
        self.prob = problem
        self.dynamics = dynamics

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


class Point_Dynamic_Error_Jacobian():
    def __init__(self, index, problem, dynamics_jac):
        self.t = index
        self.prob = problem
        # dynamics_jac evaluats the jacobian of accelation = f(q, v, u)
        self.dynamics_jac = dynamics_jac
        #the jacobian of the positon error is determined by postion and  velocity
        #And it is not influenced by the dynamic_jac, which is associated with acc
        #This is a constant so can be set before run time
        self.dim_q_jac_lst = [-1, -0.5*self.prob["dt"], 1, -0.5*self.prob["dt"]]
        self.q_jac_lst = self.dim_q_jac_lst * self.prob['qdim']

    def __call__(traj):
        #returns an array with shape (2*qdim, ||traj||)
        #the jacobian of the positon error is determined by postion and  velocity
        #And it is not influenced by the dynamic_jac, which is associated with acc
        # q_jac_lst = [-1, -0.5*self.prob["dt"], 1, -0.5*self.prob["dt"]] * self.qdim
        # postion derivative

        q0, v0, u0 = get_point_q_v_u(traj, self.t, self.prob["qdim"], self.prob["udim"])
        q1, v1, u1 = get_point_q_v_u(traj, self.t+1, self.prob["qdim"], self.prob["udim"])

        a0_jac = self.dynamics_jac(q0, v0, u0)
        a1_jac = self.dynamics_jac(q1, v1, u1)

        dq0 = -0.5*self.prob["dt"]*a0_jac[:,0:self.qdim]
        dv0 = -1 - 0.5*self.prob["dt"]*a0_jac[:, self.qdim:2*self.qdim]
        du0 = -0.5*self.prob["dt"]*a0_jac[:, 2*self.qdim:2*self.qdim+self.udim]

        dq1 = -0.5*self.prob["dt"]*a1_jac[:,0:self.qdim]
        dv1 = 1 - 0.5*self.prob["dt"]*a1_jac[:, self.qdim:2*self.qdim]
        du1 = -0.5*self.prob["dt"]*a1_jac[:, 2*self.qdim:2*self.qdim+self.udim]

        v_jac = np.concatenate([dq0, dv0, du0, dq1, dv1, du1], axis=1)
        v_jac_flat = v_jac.flatten()

        point_jac_flat = np.hstack([self.q_jac_lst, v_jac_flat])
        return point_jac_flat


class Dynamic_constraints():
    def __init__(self, qdim, udim, dt, n, dynamics, dynamics_g=None):
        self.qdim = qdim
        self.udim = udim
        self.dynamics = dynamics
        self.dynamics_jac = dynamics_g
        self.dt = dt
        self.n = n
        self.point_dim = self.qdim*2+self.udim
        self.size = (self.point_dim)*self.n
        self.create_jacobian_mask()

    def create_jacobian_mask(self):
        jac_row_lst = [self.get_point_jac_mask(t) for t in range(0, self.n-1)]
        self.jac_mask = np.vstack(jac_row_lst)
        self.jac_index = np.where(self.jac_mask==True)

    def get_point_jac_mask(self, t):
        q_jac = self.get_point_q_jac_mask(t)
        v_jac = self.get_point_v_jac_mask(t)
        point_jac_mask = np.vstack([q_jac, v_jac])
        return point_jac_mask

    def get_point_q_jac_mask(self, t):
        q_jac_lst = [self.get_point_q_dim_jac_mask(t, i)
                     for i in range(self.qdim)]
        q_jac = np.vstack(q_jac_lst)
        return q_jac


    def get_point_q_dim_jac_mask(self, t, i):
        q0_i = (self.point_dim)*t + i
        q1_i = (self.point_dim)*(t+1) + i
        v0_i = q0_i + self.qdim
        v1_i = q1_i + self.qdim
        q_dim_jac = np.zeros(self.size, dtype=bool)
        q_dim_jac[[q0_i, q1_i, v0_i, v1_i]] = True
        return q_dim_jac


    def get_point_v_jac_mask(self, t):
        v_jac = np.zeros((self.qdim, self.size), dtype=bool)
        q0_index, v0_index, u0_index = get_point_index(t, self.qdim, self.udim)
        q1_index, v1_index, u1_index = get_point_index(t+1, self.qdim, self.udim)
        v_jac[:, q0_index[0]:u1_index[-1]] = True
        return v_jac


    def get_point_jac_value(self, traj, t):
        q_jac_lst = [-1, -0.5*self.dt, 1, -0.5*self.dt] * self.qdim
        # postion derivative

        q0_index, v0_index, u0_index = get_point_index(t, self.qdim, self.udim)
        q1_index, v1_index, u1_index = get_point_index(t+1, self.qdim, self.udim)
        q0 = traj[q0_index[0]:q0_index[-1]]
        v0 = traj[v0_index[0]:v0_index[-1]]
        u0 = traj[u0_index[0]:u0_index[-1]]
        q1 = traj[q1_index[0]:q1_index[-1]]
        v1 = traj[v1_index[0]:v1_index[-1]]
        u1 = traj[u1_index[0]:u1_index[-1]]

        a0_jac = self.dynamics_jac(q0, v0, u0)
        a1_jac = self.dynamics_jac(q1, v1, u1)

        dq0 = -0.5*self.dt*a0_jac[:,0:self.qdim]
        dv0 = -1 - 0.5*self.dt*a0_jac[:, self.qdim:2*self.qdim]
        du0 = -0.5*self.dt*a0_jac[:, 2*self.qdim:2*self.qdim+self.udim]

        dq1 = -0.5*self.dt*a1_jac[:,0:self.qdim]
        dv1 = 1 - 0.5*self.dt*a1_jac[:, self.qdim:2*self.qdim]
        du1 = -0.5*self.dt*a1_jac[:, 2*self.qdim:2*self.qdim+self.udim]

        v_jac = np.concatenate([dq0, dv0, du0, dq1, dv1, du1], axis=1)
        v_jac_flat = v_jac.flatten()

        point_jac_flat = np.hstack([q_jac_lst, v_jac_flat])
        return point_jac_flat


    def get_keypoints_index(self, t):
        qdim, udim = self.qdim, self.udim
        start = t*self.point_dim
        qt_index = [start, start+qdim]
        vt_index = [qt_index[-1], qt_index[-1]+qdim]
        ut_index = [vt_index[-1], vt_index[-1]+udim]
        return qt_index, vt_index, ut_index


    def dynamic_error(self, t, traj):
        q0_index, v0_index, u0_index = self.get_keypoints_index(t)
        q1_index, v1_index, u1_index = self.get_keypoints_index(t+1)
        q0 = traj[q0_index[0]:q0_index[-1]]
        v0 = traj[v0_index[0]:v0_index[-1]]
        u0 = traj[u0_index[0]:u0_index[-1]]
        q1 = traj[q1_index[0]:q1_index[-1]]
        v1 = traj[v1_index[0]:v1_index[-1]]
        u1 = traj[u1_index[0]:u1_index[-1]]

        x0 = traj[q0_index[0]:v0_index[-1]]
        x1 = traj[q1_index[0]:v1_index[-1]]
        # the deviavie respect to [q, v], not including u: x_dot = f(x, u)
        #TODO: each point is evaluted twice; this can be improved
        d0 = np.hstack([v0, self.dynamics(q0, v0, u0)])
        d1 = np.hstack([v1, self.dynamics(q1, v1, u1)])

        error = (x1 - x0) - 0.5*self.dt*(d0 + d1) #3.2 of Kelly(2017)
        return error

    def eval_g(self, traj):
        residual = [self.dynamic_error(t, traj) for t in range(0, self.n-1)]
        return np.hstack(residual)
        # return np.sum(np.hstack(residual))


    def eval_jac_g(self, X, flag):
        if flag:
            return self.jac_index
        point_jac_lst = [self.get_point_jac_value(X, t)
                         for t in range(0, self.n-1)]
        jac_flat = np.hstack(point_jac_lst)
        return jac_flat


def eval_f(X):
    #this is the cost function
    q_arr, v_arr, u_arr = X.reshape(-1, 3).T
    square = np.power(u_arr, 2)
    cost = np.sum(square)
    return cost


def eval_g(X):
    q_arr, v_arr, u_arr = X.reshape(-1, 3).T


if __name__=="__main__":
    n = 20 # number of time point
    dt = 1/n
    start = (0, 0)
    end = (1, 1)
    q_arr = np.linspace(start[0], end[0], n)
    v_arr = np.linspace(start[1], end[1], n)
    u_arr = np.zeros_like(q_arr, dtype=float)
    X_init = np.vstack([q_arr, v_arr, u_arr]).flatten("F")

    constriant = Dynamic_constraints(1, 1, dt, n, block_dymamics, block_dymamics_jac)
    g = constriant.eval_g(X_init)
    result = constriant.eval_jac_g(X_init, False)



