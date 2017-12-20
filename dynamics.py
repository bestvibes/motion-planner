import numpy as np
import mujoco_py
import numdifftools as nd
#this module implements dynamics and the jacobian and hessian of dyanmcis
# dynamics should be in the form: a = f(q, v, u)
# dynamcs_jac is the jacobian of accelation againts (q, v, u)


class Block():
    def __init__(self, qdim, udim):
        assert qdim == udim
        self.qdim = qdim
        self.udim = udim
        self.const_jac = np.hstack([np.zeros((qdim, 2*qdim )), np.eye(qdim)])

    def dynamics(self, qvu):
        u = qvu[2*self.qdim:2*self.qdim+self.udim]
        return u

    def jac_dynamics(self, qvu):
        return self.const_jac

    def jac_dynamics_sparse(self, qvu, flag):
        if flag:
            rows = np.arange(self.qdim)
            cols = np.arange(2*self.qdim, 2*self.qdim+self.udim)
            return rows, cols
        values = np.ones(self.qdim)
        return values


def make_model(model_path):
    try:
        model = mujoco_py.load_model_from_path(model_path)
        # self.sim = mujoco_py.MjSim(self.model)
    except:
        print("Failed to parse: %s" % model_path)
        raise

    if model.ngeom >0:
    # make all contacts active
        model.geom_margin[:] = np.tile(1e+10, model.geom_margin.shape)
    if model.npair >0:
    # make all contact pairs active
        model.pair_margin[:] = np.tile(1e+10, model.pair_margin.shape)
    #     # self.model = mujoco_py.MjModel(model_filename)
    return model


class Mujoco_Dynamics:
    def __init__(self, model, sim, qdim, udim):
        # self.make_mode(model_path)
        self.model = model
        self.sim = sim
        assert self.model.nq == qdim
        self.qdim = qdim
        self.udim = udim
        self.J_func = nd.Jacobian(self.dynamics)

    def dynamics(self, qvu):
        if "adouble" in qvu[0].__repr__():
            qvu_array = np.array([float(a.__str__()[0:-3]) for a in qvu])
        else:
            qvu_array = qvu
        qdim = self.qdim
        udim = self.udim
        q = qvu_array[:qdim]
        v = qvu_array[qdim:2*qdim]
        u = qvu_array[2*qdim:]

        self.sim.data.qpos[:] = q
        self.sim.data.qvel[:] = v
        self.sim.data.ctrl[:] = u

        #forward kinematcs, without time integration
        mujoco_py.functions.mj_forward(self.model, self.sim.data)
        # mujoco_py.functions.mj_step(self.model, self.sim.data)
        qacc = np.copy(self.sim.data.qacc)
        return qacc



if __name__=="__main__":

    xdata = np.reshape(np.arange(0,1,0.1),(-1,1))
    ydata = 1+2*np.exp(0.75*xdata)
    fun = lambda c: (c[0]+c[1]*np.exp(c[2]*xdata) - ydata)**2
    Jfun = nd.Jacobian(fun)


    model_path = "/home/tao/src/gym/gym/envs/mujoco/assets/inverted_pendulum.xml"

    model = make_model(model_path)
    sim = mujoco_py.MjSim(model)
    qdim = model.nq
    udim = model.nu
    import ipdb; ipdb.set_trace()  # XXX BREAKPOINT

    cart = Mujoco_Dynamics(model, sim, qdim, udim)

    np.random.seed(42)

    Jfunc = nd.Jacobian(cart.dynamics)

    for i in range(10):
        q = np.random.uniform(-2, 2, qdim)
        v = np.random.uniform(-2, 2, qdim)
        u = np.random.uniform(-2, 2, udim)
        qvu = np.hstack([q, v, u])
        acc = cart.dynamics(qvu)
        acc_jacobian = cart.J_func(qvu)
        print (acc_jacobian)


    # model = make_model(mode)


    # prob = {}
    # prob["n"] =  2 # number of trajectory point
    # prob["dt"] = 1.0/prob["n"]
    # prob["qdim"] = 1
    # prob["udim"] = 1
    #
    #
    # start = (0, 0)
    # end = (0, 0)
    # n = prob['n']
    # q_arr = np.linspace(start[0], end[0], n)
    # v_arr = np.linspace(start[1], end[1], n)
    # # q_arr = np.zeros(n)
    # # v_arr = np.zeros(n)
    # u_arr = np.ones_like(q_arr, dtype=float)*-10
    # X_init = np.vstack([q_arr, v_arr, u_arr]).flatten("F")
    #
    # c0 = Dynamics_constriant(prob, block_dymamics, block_dymamics_jac, 0)
    #
    # g = c0.eval_g(X_init)
    # jac = c0.eval_jac_g(X_init, False)
    # mask = c0.eval_jac_g(X_init, True)
    # import ipdb; ipdb.set_trace()  # XXX BREAKPOINT



