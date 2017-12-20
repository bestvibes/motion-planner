import numpy as np
import adolc
import itertools as it
import constraint

def adouble_to_numpy(x):
    x_arr = np.array([( xi.__str__()[0:-3] ) for xi in x], dtype=float)
    return x_arr


class InstanceCounterMeta(type):
    """ Metaclass to make instance counter not share count with descendants
    """
    def __init__(cls, name, bases, attrs):
        super().__init__(name, bases, attrs)
        cls._ids = it.count(1)


class Func_Adolc(object, metaclass=InstanceCounterMeta):
    def __init__(self, f, x, scaler=False):
        if not isinstance(x, list):
            x_lst = [x]
        else:
            x_lst = x
        self.id = next(self.__class__._ids)
        self.scaler = scaler
        # x = np.random.uniform(x_L, x_U)
        adolc.trace_on(self.id)
        a_x_lst = [adolc.adouble(v) for v in x_lst]
        for ax in a_x_lst:
            adolc.independent(ax)
        ay = f(*a_x_lst)
        adolc.dependent(ay)
        adolc.trace_off()

    def __call__(self, x):
        result = adolc.function(self.id, x)
        if self.scaler:
            return result[0]
        return result


class Eval_Grad_F_Adolc():
    def __init__(self, id):
        self.id = id

    def __call__(self, x):
        return adolc.gradient(self.id, x)



class Eval_Jac_G_Adolc():
    def __init__(self, id, x):
        self.id = id
        options = np.array([1,1,0,0],dtype=int)
        result = adolc.colpack.sparse_jac_no_repeat(self.id,x,options)

        self.nnzj  = result[0]
        self.rind = np.asarray(result[1],dtype=int)
        self.cind = np.asarray(result[2],dtype=int)
        self.values = np.asarray(result[3],dtype=float)

    def __call__(self, x, flag, user_data = None):
        if flag:
            return (self.rind, self.cind)
        else:
            result = adolc.colpack.sparse_jac_repeat(self.id, x, self.nnzj,
                                                     self.rind,
                                                     self.cind, self.values)
            return result[3]

# class Eval_Hessian_G_Adolc():

class Eval_h_adolc:
    def __init__(self, id, x_l_o):
        self.id = id
        #number of constraints
        self.options = np.array([0,0],dtype=int)

        result = adolc.colpack.sparse_hess_no_repeat(self.id, x_l_o, self.options)

        self.rind = np.asarray(result[1],dtype=int)
        self.cind = np.asarray(result[2],dtype=int)
        self.values = np.asarray(result[3],dtype=float)

        # need only upper left part of the Hessian
        #FIXME not sure whether this is right; maye var/2 ?
        self.mask = np.where(self.cind < x_l_o[0].size/2)
        self.nnzh = self.mask[0].size

    def __call__(self, x, lagrange, obj_factor, flag, user_data = None):
        if flag:
            return (self.rind[self.mask], self.cind[self.mask])
        else:
            x_l_o = np.hstack([x,lagrange,obj_factor])
            result = adolc.colpack.sparse_hess_repeat(self.id, x_l_o, self.rind,
                self.cind, self.values)
            return result[3][self.mask]
