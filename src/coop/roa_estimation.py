import numpy as np
from pydrake.all import MathematicalProgram, Solve, Variables
import pydrake.symbolic as sym

from .ellipsoid import quadForm, sampleFromEllipsoid


class probTIROA:
    """
    class for probabilistic RoA estimation for linear (or linearized) systems
    under (infinite horizon) TILQR control.

    Takes a configuration dict and requires passing a callback function in
    which the simulation is done.  The callback function returns the result of
    the simulation as a boolean (True = Success)

    the conf dict has the following structure

        roaConf={   "x0Star": <goal state to stabilize around>, "xBar0Max":
        <bounds that define the first (over) estimate of the RoA to sample
        from>, "S": <cost to go matrix TODO change this to V for other
        stabilizing controllers> "nSimulations": <number of simulations> }

    TODO: generalize for non LQR systems -> V instead of S
    """
    def __init__(self, roaConf, simFct):
        self.x0Star = roaConf["x0Star"]
        self.xBar0Max = roaConf["xBar0Max"]
        self.S = roaConf["S"]
        self.nSims = roaConf["nSimulations"]
        self.simClbk = simFct

        self.rhoHist = []
        self.simSuccessHist = []

        rho0 = quadForm(self.S, self.xBar0Max)
        self.rhoHist.append(rho0)

    def doEstimate(self):
        for sim in range(self.nSims):
            # sample initial state from previously estimated RoA
            x0Bar = sampleFromEllipsoid(self.S, self.rhoHist[-1])
            JStar0 = quadForm(self.S, x0Bar)  # calculate cost to go
            x0 = self.x0Star+x0Bar  # error to absolute coords

            simSuccess = self.simClbk(x0)

            if not simSuccess:
                self.rhoHist.append(JStar0)
            else:
                self.rhoHist.append(self.rhoHist[-1])

            self.simSuccessHist.append(simSuccess)

        return self.rhoHist, self.simSuccessHist


def verify_double_pendulum_rho(rho, params, S, K, taylor_deg=3,
                               lambda_deg=4, mode=2, verbose=False,
                               x_bar_eval=[np.pi, 0, 0, 0]):
    """
    params      --> parameters of pendulum and controller
    taylor_deg  --> degree of the taylor approximation
    lamda_deg   --> degree of SOS lagrange multipliers

    solve the feasibility problem in one of three modes:
    0: completely unconstrained (no actuation limits)
        -->     fastest, will overestimate actual RoA by a lot
    1: check only where controls are not saturated
        -->     mid    , will underestimate actual RoA.
        We could actually use a for this.
        Does it make sense to maximize the region in which no saturation
        occurs, i.e. the linearization is valid and the params are well?
    2: also check for saturated dynamics
        -->     slowest, but best estimate
    """
    I1 = params["I"][0]
    I2 = params["I"][1]
    m1 = params["m"][0]
    m2 = params["m"][1]
    l1 = params["l"][0]
    # l2 = params["l"][1]
    r1 = params["lc"][0]
    r2 = params["lc"][1]
    # b1 = params["b"][0]
    # b2 = params["b"][1]
    # fc1 = params["f"][0]
    # fc2 = params["f"][1]
    # tau1_max = params["tau_max"][0]
    tau2_max = params["tau_max"][1]

    g = params["g"]
    S = S  # params["S"]
    K = K  # params["K"]

    u_plus = tau2_max
    u_minus = -tau2_max

    prog = MathematicalProgram()

    # Indeterminates in error coordinates
    x_bar = prog.NewIndeterminates(4, "x_bar")
    x_bar_1 = x_bar[0]
    x_bar_2 = x_bar[1]
    xd_bar_1 = x_bar[2]
    xd_bar_2 = x_bar[3]

    # now in physical coordinates:
    # desired
    x_star = np.array([np.pi, 0, 0, 0])
    # xd_star = np.array([0, 0, 0, 0])

    # physical
    x = x_star+x_bar

    # and finally for manipulator eqs (this is not in error coords as above!):
    q = x[0:2]
    qd = x[2:4]

    q1 = q[0]
    q2 = q[1]
    qd1 = qd[0]
    qd2 = qd[1]

    # the following is based on Felix sympy implementation
    # mass matrix
    m11 = I1 + I2 + m2*l1**2 + 2*m2*l1*r2*sym.cos(q2)
    m12 = I2 + m2*l1*r2*sym.cos(q2)
    m21 = I2 + m2*l1*r2*sym.cos(q2)
    m22 = I2

    # M   = np.array([[m11,m12],
    #                 [m21,m22]])

    det_M = m22*m11-m12*m21
    M_inv = (1/det_M) * np.array([[m22, -m12],
                                  [-m21, m11]])

    # coriolis
    c11 = -2*m2*l1*r2*sym.sin(q2) * qd2
    c12 = -m2*l1*r2*sym.sin(q2) * qd2
    c21 = m2*l1*r2*sym.sin(q2) * qd1
    c22 = 0

    C = np.array([[c11, c12],
                  [c21, c22]])

    # gravity
    g1 = -m1*g*r1*sym.sin(q1) - m2*g*(l1*sym.sin(q1) + r2*sym.sin(q1+q2))
    g2 = -m2*g*r2*sym.sin(q1+q2)

    G = np.array([g1, g2])

    # b matrix
    B = np.array([[1, 0], [0, 1]])
    # B  = np.array([[0,0],[0,1]]) # acrobot
    # B  = np.array([[1,0],[0,0]]) # pendubot

    u_minus_vec = np.array([0, u_minus])
    u_plus_vec = np.array([0, u_plus])

    # nominal and saturated dynamics
    f_exp_acc = M_inv.dot(B.dot(- K.dot(x_bar)) + G - C.dot(qd))

    if mode == 2:
        f_exp_acc_minus = M_inv.dot(B.dot(u_minus_vec) + G - C.dot(qd))
        f_exp_acc_plus = M_inv.dot(B.dot(u_plus_vec) + G - C.dot(qd))

    # f_imp = M.dot(qdd) + C.dot(qd) - G - B.dot(u)

    # taylor approximation
    env = {x_bar_1: 0,
           x_bar_2: 0,
           xd_bar_1: 0,
           xd_bar_2: 0}

    # nominal and saturated fs
    qdd1_approx = sym.TaylorExpand(f=f_exp_acc[0], a=env, order=taylor_deg)
    qdd2_approx = sym.TaylorExpand(f=f_exp_acc[1], a=env, order=taylor_deg)

    if mode == 2:
        qdd1_approx_minus = sym.TaylorExpand(f=f_exp_acc_minus[0],
                                             a=env,
                                             order=taylor_deg)
        qdd2_approx_minus = sym.TaylorExpand(f=f_exp_acc_minus[1],
                                             a=env,
                                             order=taylor_deg)
        qdd1_approx_plus = sym.TaylorExpand(f=f_exp_acc_plus[0],
                                            a=env,
                                            order=taylor_deg)
        qdd2_approx_plus = sym.TaylorExpand(f=f_exp_acc_plus[1],
                                            a=env,
                                            order=taylor_deg)

    f = np.array([[qd1],
                  [qd2],
                  [qdd1_approx],
                  [qdd2_approx]])

    if mode == 2:
        f_minus = np.array([[qd1],
                            [qd2],
                            [qdd1_approx_minus],
                            [qdd2_approx_minus]])

        f_plus = np.array([[qd1],
                           [qd2],
                           [qdd1_approx_plus],
                           [qdd2_approx_plus]])

    V = x_bar.dot(S.dot(x_bar))

    Vdot = (V.Jacobian(x_bar).dot(f))[0]

    # chekcing stuff here
    Vdot_check = 2*np.dot(x_bar, np.dot(S, f.flatten()))

    f_true = np.array([qd1, qd2, f_exp_acc[0], f_exp_acc[1]])
    Vdot_true = 2*np.dot(x_bar, np.dot(S, f_true.flatten()))

    lambda_3 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()

    if mode == 2:
        Vdot_minus = (V.Jacobian(x_bar).dot(f_minus))[0]
        Vdot_plus = (V.Jacobian(x_bar).dot(f_plus))[0]

    if mode == 2:
        # u in linear range
        lambda_4 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()
        lambda_5 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()

        # uplus
        lambda_6 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()
        lambda_7 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()

        # uminus
        lambda_1 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()
        lambda_2 = prog.NewSosPolynomial(Variables(x_bar), lambda_deg)[0].ToExpression()

    epsilon = 10e-20

    # constraints
    prog.AddSosConstraint(lambda_3)
    prog.AddSosConstraint(V)

    # completely unconstrained dynamics
    if mode == 0:
        prog.AddSosConstraint(-Vdot + lambda_3*(V-rho) -
                              epsilon*x_bar.dot(x_bar))

    if mode == 2:
        nom1 = (+ K.dot(x_bar) + u_minus_vec)[1]
        # where both nom1 and nom2 are < 0, the nominal dynamics have to
        # be fullfilled
        nom2 = (- K.dot(x_bar) - u_plus_vec)[1]
        # where this is < 0, the negative saturated dynamics have to
        # be fullfilled
        neg = (- K.dot(x_bar) - u_minus_vec)[1]
        # where this is < 0, the positive saturated dynamics have to
        # be fullfilled
        pos = (+ K.dot(x_bar) + u_plus_vec)[1]

        prog.AddSosConstraint(-Vdot + lambda_3*(V-rho) +
                              lambda_4*nom1 + lambda_5*nom2 -
                              epsilon*x_bar.dot(x_bar))

        prog.AddSosConstraint(lambda_4)
        prog.AddSosConstraint(lambda_5)

        # neg saturation
        prog.AddSosConstraint(-Vdot_minus + lambda_1*(V - rho) +
                              lambda_2*neg - epsilon*x_bar.dot(x_bar))
        prog.AddSosConstraint(lambda_1)
        prog.AddSosConstraint(lambda_2)

        # pos saturation
        prog.AddSosConstraint(-Vdot_plus + lambda_6*(V - rho) +
                              lambda_7*pos - epsilon*x_bar.dot(x_bar))
        prog.AddSosConstraint(lambda_6)
        prog.AddSosConstraint(lambda_7)

    result = Solve(prog)

    # print(prog)

    if verbose:
        env = {x_bar_1: x_bar_eval[0],
               x_bar_2: x_bar_eval[1],
               xd_bar_1: x_bar_eval[2],
               xd_bar_2: x_bar_eval[3]}

        print("-K(xBar): ")
        print(-K.dot(x_bar)[1].Evaluate(env))
        print("xBar: ")
        print(sym.Expression(x_bar[0]).Evaluate(env))
        print(sym.Expression(x_bar[1]).Evaluate(env))
        print(sym.Expression(x_bar[2]).Evaluate(env))
        print(sym.Expression(x_bar[3]).Evaluate(env))
        print("dotX (approximated) ")
        print(qd1.Evaluate(env))
        print(qd2.Evaluate(env))
        print(qdd1_approx.Evaluate(env))
        print(qdd2_approx.Evaluate(env))
        print("dotX (true) ")
        print(qd1.Evaluate(env))
        print(qd2.Evaluate(env))
        print(f_exp_acc[0].Evaluate(env))
        print(f_exp_acc[1].Evaluate(env))
        print("V")
        print(V.Evaluate(env))
        print("Vdot (approximated)")
        print(Vdot.Evaluate(env))
        print("Vdot check (approximated)")
        print(Vdot_check.Evaluate(env))
        print("Vdot true")
        print(Vdot_true.Evaluate(env))
        print("S")
        print(S)

    return result.is_success()


def bisect_and_verify(params, S, K, hyper_params, rho_min=0,
                      rho_max=20, maxiter=20, verbose=False):
    """
    Simple bisection root finding for finding the RoA using the feasibility
    problem.
    Probably makes sense to vary rho_min=0,rho_max=20,maxiter=20
    """
    for i in range(maxiter):
        # np.random.uniform(rho_min,rho_max)
        rho_probe = rho_min+(rho_max-rho_min)/2
        res = verify_double_pendulum_rho(rho_probe,
                                         params,
                                         S,
                                         K,
                                         taylor_deg=hyper_params["taylor_deg"],
                                         lambda_deg=hyper_params["lambda_deg"],
                                         mode=hyper_params["mode"])
        if verbose:
            print("---")
            print("rho_min:   "+str(rho_min))
            print("rho_probe: "+str(rho_probe)+" verified: "+str(res))
            print("rho_max:   "+str(rho_max))
            print("---")
        if res:
            rho_min = rho_probe
        else:
            rho_max = rho_probe

    return rho_min
