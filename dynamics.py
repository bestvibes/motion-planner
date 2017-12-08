import numpy as np
#this module implements dynamics and the jacobian and hessian of dyanmcis


def block_dymamics(q, v, u):
    return u


def block_dymamics_jac(q, v, u):
    jac = np.hstack([np.zeros_like(q), np.zeros_like(v), np.ones_like(u)])
    jac_2d = jac.reshape(1, -1)
    return jac_2d




if __name__=="__main__":
    prob = {}
    prob["n"] =  2 # number of trajectory point
    prob["dt"] = 1.0/prob["n"]
    prob["qdim"] = 1
    prob["udim"] = 1


    start = (0, 0)
    end = (0, 0)
    n = prob['n']
    q_arr = np.linspace(start[0], end[0], n)
    v_arr = np.linspace(start[1], end[1], n)
    # q_arr = np.zeros(n)
    # v_arr = np.zeros(n)
    u_arr = np.ones_like(q_arr, dtype=float)*-10
    X_init = np.vstack([q_arr, v_arr, u_arr]).flatten("F")

    c0 = Dynamics_constriant(prob, block_dymamics, block_dymamics_jac, 0)

    g = c0.eval_g(X_init)
    jac = c0.eval_jac_g(X_init, False)
    mask = c0.eval_jac_g(X_init, True)
    import ipdb; ipdb.set_trace()  # XXX BREAKPOINT



