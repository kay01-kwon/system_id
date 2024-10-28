from acados_template import AcadosModel
import tools
import casadi as cs

class QuadModel:
    def __init__(self, J, l, C_moment, model_description):
        '''
        Constructor for QuadModel
        :param J: Put np.diag([Jxx, Jyy, Jzz])
        :param l: arm length
        :param C_moment: Coefficient of moment
        :param model_description: '+' or 'x'
        '''

        # Model name and create AcadosModel object.
        self.model_name = 'Quadrotor_model'
        self.model = AcadosModel()

        self.l = l

        # Get parameters
        # moment of inertia, lift coefficient, moment coefficient
        self.J = J
        self.C_moment = C_moment

        # Model description ('x' or '+')
        self.model_description = model_description

        # Casadi: Assign x (state)
        self.q = cs.MX.sym('q',4)   # quaternion            qw qx qy qz
        self.w = cs.MX.sym('w',3)   # angular velocity      wx wy wz
        self.x = cs.vertcat( self.q, self.w)     #state
        self.x_dim = 7

        # Casadi: Assign u (control input: rotor speed)
        self.u1 = cs.MX.sym('u1')
        self.u2 = cs.MX.sym('u2')
        self.u3 = cs.MX.sym('u3')
        self.u4 = cs.MX.sym('u4')
        self.u = cs.vertcat(self.u1, self.u2, self.u3, self.u4)
        self.u_dim = 4

        # Casadi: Assign xdot (The differentiation of the state)
        self.dqdt = cs.MX.sym('dqdt',4)     # dq_w/dt dq_x/dt dq_y/dt dq_z/dt
        self.dwdt = cs.MX.sym('dwdt',3)     # dw_x/dt dw_y/dt dw_z/dt
        self.xdot = cs.vertcat( self.dqdt, self.dwdt)      # dpdt dvdt dqdt dwdt


    def get_acados_model(self):
        '''
        Set acados model and the return acados model of the quadrotor
        :return: model
        '''

        # Set explicit dynamics and kinematics
        self.f_expl = cs.vertcat(self.q_kinematics(), self.w_dynamics())

        self.f_impl = self.xdot - self.f_expl

        self.model.f_expl_expr = self.f_expl
        self.model.f_impl_expr = self.f_impl
        self.model.x = self.x
        self.model.xdot = self.xdot
        self.model.u = self.u
        self.model.name = self.model_name
        return self.model

    def q_kinematics(self):
        '''
        q kinematics
        :return: dqdt (dqdt = 0.5 * w otimes q)
        '''
        w_quat_form = cs.vertcat(0, self.w)       # [0 w]

        # dqdt = 1/2 * q otimes [0 w]
        dqdt = 0.5*tools.otimes(self.q, w_quat_form)
        return dqdt

    def w_dynamics(self):
        '''
        w_dynamics
        :return: dwdt
        '''

        # Get moment of inertia
        Jxx = self.J[0]
        Jyy = self.J[1]
        Jzz = self.J[2]

        # Convert Four rotor thrusts to moment
        M_x, M_y, M_z = tools.thrust2moment(self.model_description, self.u, self.l, self.C_moment)

        # Get angular acceleration due to the moment
        m_vec = cs.vertcat(M_x/Jxx , M_y/Jyy, M_z/Jzz)

        # Get angular velocity
        w_x = self.w[0]
        w_y = self.w[1]
        w_z = self.w[2]

        # inertial effect = w x (J*w)
        inertial_effect = cs.vertcat((Jzz-Jyy)/Jxx*w_y*w_z,
                                     (Jxx-Jzz)/Jyy*w_x*w_z,
                                     (Jyy-Jxx)/Jzz*w_x*w_y)

        # attitude dynamics
        # dwdt = [M_x/Jxx, M_y/Jyy, M_z/Jzz]^T - w x (J*w)
        dwdt = m_vec - inertial_effect
        return dwdt






