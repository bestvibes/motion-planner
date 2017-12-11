#!/usr/bin/python3.6
import pyipopt
import numpy as np
import dynamics as dy
import cost
import constraint
import pylab as plt
import numdifftools as nd
import util
import time
import mujoco_py
import click

np.set_printoptions(threshold=np.nan)

@click.command()
@click.option('--scenario', type=str, default='', help='name of scenario script')
@click.option('--traj_filename', type=str, default='/tmp/trajectory.pkl', help='filename of the solution trajectory')
def main():
    prob = {}
    prob["n"] = 20
    prob["qdim"] = 2
    prob["udim"] = 1
    prob["dt"] = 3.0/( prob["n"]-1)

    #cartslider: [-1, 1]
    #pole hinge: [-0.5pi ,0.5pi]
    #ctrl range: [-3, 3]
    p_L = [-20, -10*np.pi, -10, -3*np.pi, -3]
    p_U = [20, 10*np.pi, 10, 3*np.pi,  3]

    x_L =  np.tile(p_L, prob["n"])
    x_U =  np.tile(p_U, prob["n"])

    qdim = prob['qdim']
    start = np.array([0]*qdim+[0]*qdim)
    # end = np.array([0]*qdim+[0]*qdim)
    end = np.array([0, np.pi/2] +[0]*qdim)

    n = prob["n"]
    q_v_arr_lst = [np.linspace(start[i], end[i], n) for i in range(2*prob['qdim'])]
    u_arr = np.zeros((prob["udim"], n))
    # u_arr = np.ones((prob["udim"], n))*2
    X_2d = np.vstack([q_v_arr_lst, u_arr])

    X_init = X_2d.T.flatten()

    ctrl_cost = cost.Control_Cost(prob)

    g = ctrl_cost.eval_grad_f(X_init)

    c1 = constraint.Goal_constriant(prob, start, 0)
    c1_g = c1.eval_g
    # c1_jac_g = c1.eval_jac_g

    c2 = constraint.Goal_constriant(prob, end, n-1)
    c2_g = c2.eval_g
    c_index_lst =[c1.get_indexes(), c2.get_indexes()]


    D_factory= constraint.Dynamics_constriant

    model_path = "/home/tao/src/gym/gym/envs/mujoco/assets/inverted_pendulum.xml"

    model = dy.make_model(model_path)
    sim = mujoco_py.MjSim(model)
    qdim = model.nq
    udim = model.nu

    cart = dy.Mujoco_Dynamics(model, sim, qdim, udim)
    dynamics = cart.dynamics
    # block = dy.Block(prob["qdim"], prob["udim"])
    # dynamics = block.dynamics
    # jac_dynamics = block.jac_dynamics
    #
    #
    #
    d_const_lst = [D_factory(prob, dynamics, t) for t in range(n-1)]

    # d0 = d_const_lst[0]

    # J_func = nd.Jacobian(d0.eval_g)
    # d_const_lst = [D_factory(prob, dynamics, jac_dynamics, t)
    #                       for t in range(2)]
    d_eval_g_lst = [c.eval_g for c in d_const_lst]
    d_index_lst = [c.get_indexes() for c in d_const_lst]

    # d_eval_jac_lst = [c.eval_jac_g for c in d_const_lst]

    eval_g_lst = [c1_g, c2_g] + d_eval_g_lst
    # eval_g_lst = [c1_g, c2_g]
    indexes_lst = c_index_lst +  d_index_lst
    x_L_lst = [x_L[i] for i in indexes_lst]
    x_U_lst = [x_U[i] for i in indexes_lst]
    eval_jac_g_lst = [constraint.Sparse_Jacobian(g, x_L_lst[i], x_U_lst[i])
                    for (i, g) in enumerate(eval_g_lst)]


    eval_g = constraint.Stacked_Constriants(eval_g_lst, indexes_lst)
    # J_rows, J_cols = util.get_func_sparse_pattern(eval_g, x_L, x_U)

    eval_jac_g = constraint.Stacked_Constriants_Jacobian(eval_g_lst,
                                                         eval_jac_g_lst,
                                                         indexes_lst)
    # eval_jac_g = constraint.Sparse_Jacobian_offline(eval_g, J_rows, J_cols)


    nvar = X_init.size
    # ncon = eval_g.get_num_constraints(X_init)
    ncon = eval_g(X_init).size

    g_L = np.zeros(ncon)
    g_U = np.zeros(ncon)

    # np.random.seed(43)
    # X_init = np.random.uniform(x_L, x_U, x_L.size)

    nnzj = eval_jac_g(X_init, True)[0].size
    # nnzj = J_rows.size

    g = eval_g(X_init)
    # jac = eval_jac_g(X_init, False)
    # jac_n = eval_jac_g_n(X_init, False)
    # mask = eval_jac_g(X_init, True)
    mask_n = eval_jac_g(X_init, True)
    # print ( mask, mask_n )
    print ( mask_n)
    nlp = pyipopt.create(nvar, x_L, x_U, ncon, g_L, g_U, nnzj, 0, ctrl_cost.eval_f,
                        ctrl_cost.eval_grad_f, eval_g, eval_jac_g)
    output, zl, zu, constraint_multipliers, obj, status = nlp.solve(X_init)
    output_2d = output.reshape(n, -1)
    return output_2d, prob
    #



if __name__ == "__main__":

    start_time = time.time()
    output_2d, prob = main()
    print("--- %s seconds ---" % (time.time() - start_time))

    print(output_2d)
    Q = output_2d[:, 1]
    V = output_2d[:, prob["qdim"]+1]
    # U = output_2d[:, 4:]

    plt.plot(Q, V)
    plt.show()
