from acados_template import AcadosOcp, AcadosOcpSolver
from quad_attitude_model import QuadModel
import scipy.linalg
import numpy as np
import tools

# Initial state
X0 = np.array([
    1, 0.0, 0.0, 0.0,     # quaternion
    0.0, 0.0, 0.0           # angular velocity
])


class OcpSolver():
    def __init__(self, u_min = 0.78, u_max = 2.5, 
                 n_nodes = 10, t_horizon = 1,
                 C_T = 1.481e-07, C_M = 2.01309304e-09):
        '''
        Constructor for OcpSolver
        :param u_min: minimum rotor thrust (N)
        :param u_max: maximum rotor thrust (N)
        :param n_nodes: Number of nodes for NMPC
        :param T_horizon: Prediction horizon
        '''

        # Create AcadosOcp
        self.ocp = AcadosOcp()

        self.l_ = 0.340 *np.sqrt(2)/2
        self.C_T = C_T
        self.C_M = C_M

        # Object generation
        quad_model_obj = QuadModel(J =np.array([0.026, 0.026, 0.040]),
                                l = self.l_,
                                C_T = self.C_T,
                                C_M = self.C_M,
                                model_description = 'x')

        # Get Quad model from the quad_model_obj
        self.model = quad_model_obj.get_acados_model()

        # Set ocp model
        self.ocp.model = self.model

        # Set min max value of the rotor speed
        self.u_min = u_min
        self.u_max = u_max

        self.u_prev = np.zeros((4,))

        # Set horizon
        self.N = n_nodes
        self.ocp.dims.N = self.N
        self.T_horizon = t_horizon

        # Get the dimension of state, input, y and y_e (terminal)
        self.nx = self.model.x.rows()
        self.nu = self.model.u.rows()
        self.ny = self.nx + self.nu
        self.ny_e = self.nx

        # Set ocp cost
        self.set_ocp_cost()

        # Set constraint for state and input
        self.set_ocp_constraints()

        # Set solver options
        self.set_ocp_solver_options()

        # Create ocp solver
        self.acados_ocp_solver = AcadosOcpSolver(self.ocp)

    def set_ocp_cost(self):
        '''
        Set OCP cost
        :return:
        '''
        # cost Q
        # qw qx qy qz
        # wx wy wz
        self.Q_mat = np.diag([0, 0.5, 0.5, 0.5,
                              0.1, 0.1, 0.01])

        # cost R:
        # u1, u2, u3, u4 (RPM)
        self.R_mat = 0.001*np.diag([1.0, 1.0, 1.0, 1.0])

        # Set cost type for OCP
        self.ocp.cost.cost_type = 'LINEAR_LS'
        self.ocp.cost.cost_type_e = 'LINEAR_LS'

        # Cost setup for state
        self.ocp.cost.Vx = np.zeros((self.ny, self.nx))
        self.ocp.cost.Vx[:self.nx, :self.nx] = np.eye(self.nx)
        self.ocp.cost.Vx_e = np.eye(self.nx)

        # Cost setup for control input
        self.ocp.cost.Vu = np.zeros((self.ny, self.nu))
        self.ocp.cost.Vu[-4:, -4:] = np.eye(self.nu)

        # Weight setup
        self.ocp.cost.W = scipy.linalg.block_diag(self.Q_mat, self.R_mat)
        self.ocp.cost.W_e = self.Q_mat

        # Reference setup
        self.ocp.cost.yref = np.concatenate((X0, np.zeros(4)))
        self.ocp.cost.yref_e = X0

    def set_ocp_constraints(self):
        '''
        Set constraints
        :return: None
        '''
        # Constraint on initial state
        self.ocp.constraints.x0 = X0

        # Constraint on control input
        self.ocp.constraints.lbu = np.array([self.u_min] * 4)
        self.ocp.constraints.ubu = np.array([self.u_max] * 4)
        self.ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    def set_ocp_solver_options(self):
        '''
        Set solver options
        :return: None
        '''
        self.ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
        self.ocp.solver_options.hessian_solver = "GAUSSIAN_NEWTON"
        self.ocp.solver_options.integrator_type = "ERK"
        self.ocp.solver_options.print_level = 0     # Do not print out
        self.ocp.solver_options.nlp_solver = "SQP_RTI"

        self.ocp.solver_options.tf = self.T_horizon

    def ocp_solve(self, state, ref):
        '''
        Set ocp solver (State and reference)
        :param state: Initial state of p_xyz, v_xyz, q_wxyz, w_xyz
        :param ref: q_wxyz, w_xyz
        :return: u
        '''

        y_ref = np.concatenate((ref, np.zeros((self.nu,))))
        y_ref_N = ref

        # Fill initial state
        self.acados_ocp_solver.set(0,"lbx", state)
        self.acados_ocp_solver.set(0, "ubx", state)

        for stage in range(self.ocp.dims.N):
            self.acados_ocp_solver.set(stage, "y_ref", y_ref)
        self.acados_ocp_solver.set(self.ocp.dims.N,"y_ref", y_ref_N)

        status = self.acados_ocp_solver.solve()

        u = self.acados_ocp_solver.get(0,"u")

        moment = tools.thrust2moment('x', u, self.l_, self.C_T, self.C_M)

        print("Moment x: ", moment[0])
        print("Moment y: ", moment[1])
        print("Moment z: ", moment[2])
        print("\n")

        # print("ref w: ", ref[0])
        # print("ref x: ", ref[1])
        # print("ref y: ", ref[2])
        # print("ref z: ", ref[3])
        # print("\n")

        self.u_prev = u

        return status, u