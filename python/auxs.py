from math import pi

# ----------------------------------------------------------------------
#   CONVERSOES DE UNIDADE
# ----------------------------------------------------------------------

TWO_PI = 2.0 * pi

def rads_to_rpm(omega):
    """Converte velocidade angular de rad/s para rpm."""
    return omega * 60.0 / TWO_PI


def rpm_to_rads(rpm):
    """Converte velocidade de rpm para rad/s."""
    return rpm * TWO_PI / 60.0