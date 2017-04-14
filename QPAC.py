import math



def findT(epsilon, delta, gamma, states, actions):
    epsilon1 = epsilon * (1-gamma) / 9.0
    kappa = 1 / ((1 - gamma) * epsilon1)
    m = math.log(3*states*actions*(1+states*actions*kappa/delta)) / (2 * epsilon1**2 * (1 - gamma)**2) 
    t = (states*actions) / ((1-gamma)**8 * epsilon**4) * math.log(1/delta) * math.log(1/(epsilon*(1-gamma))) * math.log((states*actions)/(delta*epsilon*(1-gamma)))

    print("epsilon1:", epsilon1)
    print("t:", t)
    print("m:", m)


findT(1, .99, 0.001, 8192, 4)
