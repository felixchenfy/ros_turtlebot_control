import numpy as np


class PidController(object):
    ''' PID controller '''

    def __init__(self, T, P=0.0, I=0.0, D=0.0):
        ''' Arguments
        T {float}: Control period. Unit: second.
            This is the inverse of control frequency.
        P {float or np.array}: Proportional control coefficient.
        I {float or np.array}: Integral control coefficient.
        D {float or np.array}: Differential control coefficient.
        '''

        # -- Check input data
        b1 = all(isinstance(d, float) for d in [P, I, D])
        b2 = all(isinstance(d, np.ndarray) for d in [P, I, D])
        if not b1 and not b2:
            pid_coef_types = [type(d) for d in [P, I, D]]
            err_msg = "PidController: Data type of P,I,D coefficients "\
                "are wrong: " + str(pid_coef_types)
            raise RuntimeError(err_msg)
        dim = 1 if b1 else len(P)  # Dimension of the control variable

        # -- Initialize arguments.
        self._T = T
        self._P = np.zeros(dim)+P
        self._I = np.zeros(dim)+I
        self._D = np.zeros(dim)+D
        self._err_inte = np.zeros(dim)  # Integration error.
        self._err_prev = np.zeros(dim)  # Previous error.

    def compute(self, err):
        ''' Given the error, compute the desired control value . '''

        ctrl_val = 0
        err = np.array(err)

        # P
        ctrl_val += np.dot(err, self._P)

        # I
        self._err_inte += err
        ctrl_val += self._T * np.dot(self._err_inte, self._I)

        # D
        ctrl_val += np.dot(err-self._err_prev, self._D) / self._T

        self._err_prev = err
        return ctrl_val

