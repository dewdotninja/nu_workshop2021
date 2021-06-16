'''
lag3_plantsim.py
dew.ninja  June 2021
Perform step response of lag3 transfer function
P(s) = 1/(s+1)**3

with specified period T

'''
import time
T = 0.05
datasize = 200

a = 2+T
b = T-2
y_states = [0.0]*6
u_states = [0.0]*6

t_current = 0
t_previous = 0
t = 0 
dt = 0
u = 1

def lag3(a,b,T, u, u_states, y_states):
    for k in range(3):
        y_states[2*k] = y_states[2*k+1]
        u_states[2*k] = u_states[2*k+1]
        if k == 0:
            u_states[2*k+1] = u
        else:
            u_states[2*k+1] = y_states[2*k-1]
        y_states[2*k+1] = (1/a)*(-b*y_states[2*k]+T*(u_states[2*k+1]+u_states[2*k]))
    return y_states[5]
                                 

for i in range(datasize):
    if i==0:
        print("datamat = np.array([")

    while dt < T:  # block execution until T expires
        t_current = time.ticks_ms()
        dt = t_current - t_previous
        
    y = lag3(a,b,T,u, u_states, y_states)
    print("[{},{},{},{}],".format(round(t,2),u,y,u))
    if i==datasize-1:
        print("])")
    t+=T
    