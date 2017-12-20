import numpy as np

class Control_Cost():
    def __init__(self, prob):
        # self.prob = prob
        self.n = prob["n"]
        self.qdim = prob["qdim"]
        self.udim = prob["udim"]
        self.pdim = 2*self.qdim+self.udim

    def __call__(self, X):
        x_2d = X.reshape(self.n, self.pdim)
        u = x_2d[:, 2*self.qdim:]
        value = np.sum(np.power(u, 2))
        return value


    def eval_grad_f(self, X):
        # # prob = self.prob
        # n = prob["n"]
        # qdim = prob["qdim"]
        # udim = prob["udim"]
        x_2d = X.reshape(self.n, self.pdim)
        u = x_2d[:, 2*self.qdim:]

        grad_2d = np.zeros_like(x_2d)
        grad_2d[:, 2*self.qdim:] = u
        grad = grad_2d.flatten()
        return grad


class Stacked_Costs(object):

    """summing a list of costs, each only takes part of the input"""

    def __init__(self, eval_f_lst, indexes_lst):
        """TODO: to be defined1.

        :eval_f_lst: TODO
        :indexes_lst: TODO

        """
        # self._eval_f_lst = eval_f_lst
        # self._indexes_lst = indexes_lst
        self._f_i_pair_lst = zip(eval_f_lst, indexes_lst)

    def eval_f(self, X):
        cost_lst = [func(X[indexes]) for (func, indexes) in self._f_i_pair_lst]
        return np.sum(cost_lst)




