import numpy as np
from scipy.optimize import minimize
import math

def motion_profile(args):
    vs,J,Fmax,L,t = args
    s=lambda x: (vs*t*(2*x[0]+x[1])+J*x[0]*math.pow(t,3)*(2*x[0]+x[1])*(x[0]+x[1])+(vs+J*x[0]*math.pow(t,2)*(x[0]+x[1]))*(2*x[2]+x[3])*t-J*x[2]*math.pow(t,3)*(2*x[2]+x[3])*(x[2]+x[3])-L)**2
    return s

def cons(args):
    Fmax,ve,Amax,J,vs,t = args
    cons = ({'type':'ineq',
             'fun':lambda x:np.array([Amax/J - x[0]*t,
                                      x[0],
                                      x[1],
                                      x[2],
                                      x[3],
                                      Amax/J - x[2]*t,
                                      ve-vs-J*(x[0]*t)**2 -J*x[0]*t*x[1]*t + J*x[2]*t*x[3]*t + J*(x[2]*t)**2,
                                      Fmax-ve-J*x[2]*math.pow(t,2)*(x[2]+x[3]),
                                      vs + J*(x[0]*t)**2 + J*x[0]*t*x[1]*t - J*x[2]*t*x[3]*t -J*(x[2]*t)**2])})
    return cons

def optimization(args,args1,x0):
    conditions = cons(args1)
    res = minimize(motion_profile(args),x0,method='SLSQP',constraints = conditions)

    time_list = []
    time_list.append(res.x[0])
    time_list.append(res.x[1])
    time_list.append(res.x[2])
    time_list.append(res.x[3])
    return time_list

'''args=(40,80000,80,5,0.002)
args1=(80,40,2000,80000,40,0.002)
x0=np.array((1,0,0,0))
res = optimization(args,args1,x0)
print(res)'''
