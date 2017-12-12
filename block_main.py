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

np.set_printoptions(threshold=np.nan)

def main():
    prob = {}
    prob["n"] = 20
    prob["qdim"] = 2
    prob["udim"] = 2
    prob["dt"] = 1.0/( prob["n"]-1)

    qdim = prob['qdim']
    start = np.array([0]*qdim+[0]*qdim)
    end = np.array([10]*qdim+[0]*qdim)

    n = prob["n"]
    q_v_arr_lst = [np.linspace(start[i], end[i], n) for i in range(2*prob['qdim'])]
    # u_arr = np.zeros((prob["udim"], n))
    u_arr = np.ones((prob["udim"], n))*2
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
    block = dy.Block(prob["qdim"], prob["udim"])
    dynamics = block.dynamics
    # jac_dynamics = block.jac_dynamics
    #
    #
    #
    d_const_lst = [D_factory(prob, dynamics,  t)
                          for t in range(n-1)]

    # d0 = d_const_lst[0]

    # J_func = nd.Jacobian(d0.eval_g)
    # d_const_lst = [D_factory(prob, dynamics, jac_dynamics, t)
    #                       for t in range(2)]
    d_eval_g_lst = [c.eval_g for c in d_const_lst]
    d_index_lst = [c.get_indexes() for c in d_const_lst]

    # d_eval_jac_lst = [c.eval_jac_g for c in d_const_lst]

    x_L = np.ones(X_init.size)*-100
    x_U = np.ones(X_init.size)*100


    # eval_g_lst = [c1_g, c2_g] + d_eval_g_lst
    eval_g_lst = [c1_g, c2_g]
    indexes_lst = c_index_lst +  d_index_lst
    x_L_lst = [x_L[i] for i in indexes_lst]
    x_U_lst = [x_U[i] for i in indexes_lst]
    eval_jac_g_lst = [constraint.Sparse_Jacobian_Adolc(g, x_L_lst[i], x_U_lst[i], i)
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

    print (output_2d)
    Q = output_2d[:, 0]
    V = output_2d[:, prob["qdim"]]
    # U = output_2d[:, 4:]

    plt.plot(Q, V)
    plt.show()
