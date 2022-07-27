import numpy as np

from .lqr import lqr
from ...model.symbolic_plant import SymbolicDoublePendulum


class LQRController():
    def __init__(self,
                 mass=[0.5, 0.6],
                 length=[0.3, 0.2],
                 com=[0.3, 0.2],
                 damping=[0.1, 0.1],
                 coulomb_fric=[0.0, 0.0],
                 gravity=9.81,
                 inertia=[None, None],
                 torque_limit=[0.0, 1.0]):

        self.mass = mass
        self.length = length
        self.com = com
        self.damping = damping
        self.cfric = coulomb_fric
        self.gravity = gravity
        self.inertia = inertia
        self.torque_limit = torque_limit

        self.splant = SymbolicDoublePendulum(mass=self.mass,
                                             length=self.length,
                                             com=self.com,
                                             damping=self.damping,
                                             gravity=self.gravity,
                                             coulomb_fric=self.cfric,
                                             inertia=self.inertia,
                                             torque_limit=self.torque_limit)

        # set default parameters
        self.set_goal()
        self.set_parameters()
        self.set_cost_parameters()

    def set_goal(self, x=[np.pi, 0., 0., 0.]):
        y = x.copy()
        y[0] = y[0] % (2*np.pi)
        y[1] = (y[1] + np.pi) % (2*np.pi) - np.pi
        self.xd = np.asarray(y)

    def set_parameters(self, failure_value=np.nan,
                       cost_to_go_cut=15.):
        self.failure_value = failure_value
        self.cost_to_go_cut = cost_to_go_cut

    def set_cost_parameters(self,
                            p1p1_cost=1.,
                            p2p2_cost=1.,
                            v1v1_cost=1.,
                            v2v2_cost=1.,
                            p1p2_cost=0.,
                            v1v2_cost=0.,
                            p1v1_cost=0.,
                            p1v2_cost=0.,
                            p2v1_cost=0.,
                            p2v2_cost=0.,
                            u1u1_cost=0.01,
                            u2u2_cost=0.01,
                            u1u2_cost=0.):
        # state cost matrix
        self.Q = np.array([[p1p1_cost, p1p2_cost, p1v1_cost, p1v2_cost],
                           [p1p2_cost, p2p2_cost, p2v1_cost, p2v2_cost],
                           [p1v1_cost, p2v1_cost, v1v1_cost, v1v2_cost],
                           [p1v2_cost, p2v2_cost, v1v2_cost, v2v2_cost]])

        # control cost matrix
        self.R = np.array([[u1u1_cost, u1u2_cost], [u1u2_cost, u2u2_cost]])

    def set_cost_parameters_(self,
                             pars=[1., 1., 1., 1.,
                                   0., 0., 0., 0., 0., 0.,
                                   0.01, 0.01, 0.]):
        self.set_cost_parameters(p1p1_cost=pars[0],
                                 p2p2_cost=pars[1],
                                 v1v1_cost=pars[2],
                                 v2v2_cost=pars[3],
                                 p1v1_cost=pars[4],
                                 p1v2_cost=pars[5],
                                 p2v1_cost=pars[6],
                                 p2v2_cost=pars[7],
                                 u1u1_cost=pars[8],
                                 u2u2_cost=pars[9],
                                 u1u2_cost=pars[10])

    def set_cost_matrices(self, Q, R):
        self.Q = np.asarray(Q)
        self.R = np.asarray(R)

    def init(self):
        Alin, Blin = self.splant.linear_matrices(x0=self.xd, u0=[0.0, 0.0])
        self.K, self.S, _ = lqr(Alin, Blin, self.Q, self.R)

    def get_control_output(self, x, t=None):
        y = x.copy()
        y[0] = y[0] % (2*np.pi)
        y[1] = (y[1] + np.pi) % (2*np.pi) - np.pi

        y -= self.xd

        u = -self.K.dot(y)
        u = np.asarray(u)[0]

        if y.dot(np.asarray(self.S.dot(y))[0]) > self.cost_to_go_cut:
            u = [self.failure_value, self.failure_value]

        u[0] = np.clip(u[0], -self.torque_limit[0], self.torque_limit[0])
        u[1] = np.clip(u[1], -self.torque_limit[1], self.torque_limit[1])

        return u
