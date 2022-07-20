import numpy as np

from .acrobot.controller.lqr.lqr_controller import LQRController
from .acrobot.model.symbolic_plant import SymbolicDoublePendulum
from .acrobot.simulation.simulation import Simulator
from .ellipsoid import quadForm, sampleFromEllipsoid, volEllipsoid
from .roa_estimation import probTIROA

# TODO: interface for combined optimization is defined 2 times


class caprr_wrapper:
    def __init__(self, p, c, tf=5, dt=0.01):
        self.tf = tf
        self.dt = dt
        self.sim = Simulator(plant=p)
        self.controller = c

    def sim_callback(self, x0):
        """
        Callback that is passed to the probabilitic RoA estimation class
        """
        t, x, tau = self.sim.simulate(
                t0=0.0,
                x0=x0,
                tf=self.tf,
                dt=self.dt,
                controller=self.controller,
                integrator="runge_kutta")

        if np.isnan(x[-1]).all():
            return False
        else:
            return True


def najafi(plant, controller, S, n):
    x_star = np.array([np.pi, 0.0, 0.0, 0.0])
    rho = 10
    for i in range(n):
        # sample initial state from sublevel set
        # check if it fullfills Lyapunov conditions
        x_bar = sampleFromEllipsoid(S, rho)
        x = x_star+x_bar

        tau = controller.get_control_output(x)

        xdot = plant.rhs(0, x, tau)

        V = quadForm(S, x_bar)

        Vdot = 2*np.dot(x_bar, np.dot(S, xdot))

        if V < rho and Vdot > 0.0:
            # if one of the lyapunov conditions is not satisfied
            rho = V

    return rho


def najafi_direct(plant, controller, S, n):
    x_star = np.array([np.pi, 0.0, 0.0, 0.0])
    rho = 10
    for i in range(n):
        # sample initial state from sublevel set
        # check if it fullfills Lyapunov conditions
        x_bar = sampleFromEllipsoid(S, rho)
        x = x_star+x_bar

        tau = controller.get_control_output(x)

        xdot = plant.rhs(0, x, tau)

        V = quadForm(S, x_bar)

        Vdot = 2*np.dot(x_bar, np.dot(S, xdot))

        if V > rho:
            print("something is fishy")
        # V < rho is true trivially, because we sample from the ellipsoid
        if Vdot > 0.0:
            # if one of the lyapunov conditions is not satisfied
            rho = V

    return rho


class caprr_coopt_interface:
    def __init__(self, design_params, Q, R, backend="sos",
                 log_obj_fct=False, verbose=False,
                 estimate_clbk=None, najafi_evals=10000):
        """
        object for design/parameter co-optimization.
        helps keeping track of design parameters during cooptimization.

        Intended usage:
        call `design_opt_obj` within you design optimization

        `mode` is either

        `backend` must be one of the following:
        - `sos`:        unconstrained SOS problem. Fastest, but maybe not
                        a good proxy for the actual dynamics
        - `sos_con`:    sos with constraints, modelling constrained dynamics
        - `prob`:       probabilistic simulation based
        - `najafi`:     probabilistic evaluation of Lyapunov function only
        """

        # design params and controller gains. these are mutable
        self.design_params = design_params

        # already synthesize controller here to get self.S and self.K
        self._update_lqr(Q, R)

        self.backend = backend

        # history to store dicts with complete design parameters
        self.param_hist = []

        if self.backend == "sos":
            self.verification_hyper_params = {"taylor_deg": 3,
                                              "lambda_deg": 4,
                                              "mode": 0}
        if self.backend == "sos_con":
            self.verification_hyper_params = {"taylor_deg": 3,
                                              "lambda_deg": 4,
                                              "mode": 2}

        if self.backend == "prob":
            pass

        self.verbose = verbose

        # callback function called from the estimate.
        # user can define this in order to get insight during optimization
        self.estimate_clbk = estimate_clbk

        # number of evals for the najafi method
        self.najafi_evals = najafi_evals

    def combined_opt_obj(self, y_comb):
        """
        y_comb contains the following entries (in this order):
        m2,l1,l2,q11,q22,q33,q44,r11,r22
        """
        m1 = self.design_params["m"][0]
        m2 = y_comb[0]
        l1 = y_comb[1]
        l2 = y_comb[2]

        # update new design parameters
        self.design_params["m"][1] = m2
        self.design_params["l"][0] = l1
        self.design_params["l"][1] = l2
        self.design_params["lc"][0] = l1
        self.design_params["lc"][1] = l2
        self.design_params["I"][0] = m1*l1**2
        self.design_params["I"][1] = m2*l2**2

        Q = np.diag((y_comb[3],
                     y_comb[4],
                     y_comb[5],
                     y_comb[6]))

        R = np.diag((y_comb[7], y_comb[8]))

        self._update_lqr(Q, R)

        vol, _, _ = self._estimate()

        if self.verbose:
            print(self.design_params)

        return -vol

    def combined_reduced_opt_obj(self, y_comb):
        """
        y_comb contains the following entries (in this order):
        m2,l1,l2,q11&q22,q33&q44,r11&r22
        """
        m1 = self.design_params["m"][0]
        m2 = y_comb[0]
        l1 = y_comb[1]
        l2 = y_comb[2]

        # update new design parameters
        self.design_params["m"][1] = m2
        self.design_params["l"][0] = l1
        self.design_params["l"][1] = l2
        self.design_params["lc"][0] = l1
        self.design_params["lc"][1] = l2
        self.design_params["I"][0] = m1*l1**2
        self.design_params["I"][1] = m2*l2**2

        Q = np.diag((y_comb[3],
                     y_comb[3],
                     y_comb[4],
                     y_comb[4]))

        R = np.diag((y_comb[5], y_comb[5]))

        self._update_lqr(Q, R)

        vol, _, _ = self._estimate()

        if self.verbose:
            print(self.design_params)

        return -vol

    def design_opt_obj(self, y):
        """
        objective function for design optimization
        """
        m1 = self.design_params["m"][0]
        m2 = y[0]
        l1 = y[1]
        l2 = y[2]

        # update new design parameters
        self.design_params["m"][1] = m2
        self.design_params["l"][0] = l1
        self.design_params["l"][1] = l2
        self.design_params["lc"][0] = l1
        self.design_params["lc"][1] = l2
        self.design_params["I"][0] = m1*l1**2
        self.design_params["I"][1] = m2*l2**2

        # update lqr for the new parameters. K and S are also computed here.
        # here this just recomputes S and K for the new design
        self._update_lqr(self.Q, self.R)

        vol, rho_f, S = self._estimate()

        return -vol

    def lqr_param_opt_obj(self, Q, R, verbose=False):

        self._update_lqr(Q, R)

        vol, _, _ = self._estimate()

        return -vol

    def lqr_param_reduced_opt_obj(self, y_comb, verbose=False):

        Q = np.diag((y_comb[0],
                     y_comb[0],
                     y_comb[1],
                     y_comb[1]))

        R = np.diag((y_comb[2], y_comb[2]))

        self._update_lqr(Q, R)

        vol, _, _ = self._estimate()

        return -vol

    def design_and_lqr_opt_obj(self, Q, R, y, verbose=False):
        m1 = self.design_params["m"][0]
        m2 = y[0]
        l1 = y[1]
        l2 = y[2]

        # update new design parameters
        self.design_params["m"][1] = m2
        self.design_params["l"][0] = l1
        self.design_params["l"][1] = l2
        self.design_params["lc"][0] = l1
        self.design_params["lc"][1] = l2
        self.design_params["I"][0] = m1*l1**2
        self.design_params["I"][1] = m2*l2**2

        # update lqr. K and S are also contained in design_params
        self._update_lqr(Q, R)

        vol, _, _ = self._estimate()

        if verbose:
            print(self.design_params)

        return -vol

    def _estimate(self):

        if self.backend == "sos" or self.backend == "sos_con":
            rho_f = bisect_and_verify(
                    self.design_params,
                    self.S,
                    self.K,
                    self.verification_hyper_params,
                    verbose=self.verbose,
                    rho_min=0,
                    rho_max=3,
                    maxiter=12)

        if self.backend == "prob" or self.backend == "najafi":
            plant = SymbolicDoublePendulum(
                       mass=self.design_params["m"],
                       length=self.design_params["l"],
                       com=self.design_params["lc"],
                       damping=self.design_params["b"],
                       gravity=self.design_params["g"],
                       coulomb_fric=self.design_params["fc"],
                       inertia=self.design_params["I"],
                       torque_limit=self.design_params["tau_max"])

            if self.backend == "prob":
                eminem = caprr_wrapper(plant, self.controller)
                conf = {"x0Star": np.array([np.pi, 0.0, 0.0, 0.0]),
                        "S": self.S,
                        "xBar0Max": np.array([+0.5, +0.0, 0.0, 0.0]),
                        "nSimulations": 250
                        }

                # create estimation object
                estimator = probTIROA(conf, eminem.sim_callback)
                # set random seed for reproducability
                np.random.seed(250)
                # do the actual estimation
                rho_hist, simSuccesHist = estimator.doEstimate()
                rho_f = rho_hist[-1]

            if self.backend == "najafi":
                # np.random.seed(250)
                rho_f = najafi(plant,
                               self.controller,
                               self.S,
                               self.najafi_evals)
                # print(rho_f)

        vol = volEllipsoid(rho_f, self.S)

        if self.estimate_clbk is not None:
            self.estimate_clbk(self.design_params, rho_f, vol, self.Q, self.R)

        return vol, rho_f, self.S

    def _update_lqr(self, Q, R):
        self.controller = LQRController(
                mass=self.design_params["m"],
                length=self.design_params["l"],
                com=self.design_params["lc"],
                damping=self.design_params["b"],
                coulomb_fric=self.design_params["fc"],
                gravity=self.design_params["g"],
                inertia=self.design_params["I"],
                torque_limit=self.design_params["tau_max"])

        self.Q = Q
        self.R = R

        self.controller.set_cost_parameters(p1p1_cost=self.Q[0][0],
                                            p2p2_cost=self.Q[1][1],
                                            v1v1_cost=self.Q[2][2],
                                            v2v2_cost=self.Q[3][3],
                                            p1p2_cost=self.Q[0][1],
                                            p2v1_cost=self.Q[1][2],
                                            p2v2_cost=self.Q[2][3],
                                            u1u1_cost=self.R[0][0],
                                            u2u2_cost=self.R[1][1],
                                            u1u2_cost=self.R[0][1],
                                            )
        self.controller.init()
        self.K = np.array(self.controller.K)
        self.S = np.array(self.controller.S)

    # def log(self):
    #     pass

    # def set_design_params(self,params):
    #     pass

    # def set_lqr_params(self,Q,R):
    #     pass

