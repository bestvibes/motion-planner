#!/usr/bin/python3.6
import pyipopt
import numpy as np
import dynamics as dy
import cost
import constraint
import adolc_appro as aa
import pylab as plt
import util
import time
import mujoco_py
import click
# import pickle

np.set_printoptions(threshold=np.nan)

@click.command()
@click.option('--traj_filename', type=str, default='/tmp/trajectory.pkl', help='filename of the solution trajectory')
def main(traj_filename):
    prob = {}
    prob["n"] = 10
    prob["qdim"] = 2
    prob["udim"] = 1
    # prob["dt"] = 0.1/( prob["n"]-1)
    prob["dt"] = 0.1

    p_L = [-2, -1*np.pi, -2, -np.pi, -3]
    p_U = [2, 1*np.pi, 2, np.pi,  3]

    x_L =  np.tile(p_L, prob["n"])
    x_U =  np.tile(p_U, prob["n"])

    qdim = prob['qdim']
    start = np.array([0]*qdim+[0]*qdim)
    start[1] = -np.pi
    start[1] = 0
    end = np.array([0]*qdim+[0]*qdim)

    n = prob["n"]
    q_v_arr_lst = [np.linspace(start[i], end[i], n) for i in range(2*prob['qdim'])]
    u_arr = np.ones((prob["udim"], n))*0.001
    X_2d = np.vstack([q_v_arr_lst, u_arr])

    X_init = X_2d.T.flatten()
    # set the control cost
    X_sample = np.random.uniform(x_L, x_U)

    #set the cost and the gradient of the cost
    ctrl_cost = cost.Control_Cost(prob)
    eval_f_adolc = aa.Func_Adolc(ctrl_cost, X_sample, scaler=True)
    eval_grad_f_adolc = aa.Eval_Grad_F_Adolc(eval_f_adolc.id)
    # eval_grad_f_adolc = aa.Func_Adolc(ctrl_cost.eval_grad_f, X_sample)


    #set the constriant function for points at specific time
    g1 = np.array([-np.pi, 0])
    g2 = np.array([0, 0])
    points = [(0, g1), (n-1, g2)]
    # points = [(n-1, end)]
    # points = [(0, start)]
    # p_index, p_g_func = [constraint.get_point_constriant(prob, t, g)
    #                      for (t, g) in points]
    # q and v of the pole
    dims = np.array([1,1+prob["qdim"]])
    p_index_g_piar = [constraint.get_point_constriant(prob, t, g, dims)
                         for (t, g) in points]
    p_index_iter, p_g_func_iter = zip(*p_index_g_piar)
    p_index_lst = list(p_index_iter)
    p_g_lst = list(p_g_func_iter)

    # p_g_adolc_lst = [aa.Func_Adolc(g, X_sample[i])
    #                       for (i, g) in p_index_g_piar]

    D_factory= constraint.Dynamics_constriant
    model_path = "/home/tao/src/gym/gym/envs/mujoco/assets/inverted_pendulum.xml"

    model = dy.make_model(model_path)
    sim = mujoco_py.MjSim(model)
    qdim = model.nq
    udim = model.nu

    cart = dy.Mujoco_Dynamics(model, sim, qdim, udim)
    dynamics = cart.dynamics

    d_index, d_g_func = constraint.get_dynamic_constriants(prob,
                                                           dynamics,
                                                           range(0, n-1))
    # d_g_adolc = aa.Func_Adolc(d_g_func, X_sample[d_index[0]])

    #all dynamcis shares the same approximation function
    # d_g_adolc_lst = [d_g_adolc for i in d_index]
    d_g_lst = [d_g_func for i in d_index]

    index_lst = p_index_lst + d_index
    eval_g_lst = p_g_lst + d_g_lst

    # X_sample_lst = [X_sample[i] for i in index_lst]
    #
    # g_adolc_x_pair = zip(eval_g_adolc_lst, X_sample_lst)
    #
    # eval_jac_adolc_lst = [aa.Eval_Jac_G_Adolc(g.id, x)
    #                 for (g, x) in g_adolc_x_pair]


    eval_g = constraint.Stacked_Constriants(eval_g_lst, index_lst)

    eval_g_adolc = aa.Func_Adolc(eval_g, X_sample)
    eval_jac_g_adolc = aa.Eval_Jac_G_Adolc(eval_g_adolc.id, X_sample)
    # eval_g_adolc = aa.Func_Adolc(eval_g, X_sample)

    # eval_jac_g = constraint.Stacked_Constriants_Jacobian(eval_g_lst   ,
    #                                                      eval_jac_lst,
    #                                                      index_lst)
    nvar = X_init.size
    ncon = eval_g(X_init).size

    eval_lagrangian = constraint.Eval_Lagrangian(ctrl_cost, eval_g)
    #x, lagrangian, obj_factor
    x_lag_lst = [X_sample, np.ones(ncon), 1]
    x_lag_arr = np.hstack(x_lag_lst)
    eval_lagrangian_adolc = aa.Func_Adolc(eval_lagrangian, x_lag_lst)

    eval_h_adolc = aa.Eval_h_adolc(eval_lagrangian_adolc.id, x_lag_arr)
    maks = eval_h_adolc(X_init, np.ones(ncon), 1,  True)
    H = eval_h_adolc(X_init, np.ones(ncon), 1, False)
    g_L = np.zeros(ncon)
    g_U = np.zeros(ncon)

    nnzj = eval_jac_g_adolc.nnzj
    nnzh = eval_h_adolc.nnzh
    nlp = pyipopt.create(nvar, x_L, x_U, ncon, g_L, g_U, nnzj, 0, eval_f_adolc ,
                        eval_grad_f_adolc, eval_g_adolc, eval_jac_g_adolc)
    # nlp = pyipopt.create(nvar, x_L, x_U, ncon, g_L, g_U, nnzj, nnzh, eval_f_adolc ,
    #                     eval_grad_f_adolc, eval_g_adolc, eval_jac_g_adolc, eval_h_adolc)

    output, zl, zu, constraint_multipliers, obj, status = nlp.solve(X_init)
    output_2d = output.reshape(n, -1)
    output.dump(traj_filename)
    return output_2d, prob



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
