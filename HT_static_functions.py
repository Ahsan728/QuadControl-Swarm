import numpy as np


def Thrust_to_PWM(Thrust):
    pwm_signal = 65535 * (-140.5e-3 + np.sqrt(140.5e-3 ** 2 - 4 * 0.409e-3 * (-0.099 - Thrust))) / (2 * 0.409e-3 * 256)
    return pwm_signal
