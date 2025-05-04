import casadi as ca


if True:
    z = ca.SX.sym('z')
    x = ca.SX.sym('x')

    g0 = ca.sin(x+z)
    g1 = ca.cos(x-z)

    g = ca.Function('g', [z,x], [g0,g1])
    G = ca.rootfinder('G','newton', g)

    print(G)

    res = G(0.1,0.1)
    print(res)

if False:
    pass
