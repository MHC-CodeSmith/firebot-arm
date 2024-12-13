import math

def inverse_kinematics(x, y, z):
    """
    Calcula os ângulos e deslocamentos das juntas para atingir uma posição cartesiana desejada.

    :param x: Coordenada x do end-effector
    :param y: Coordenada y do end-effector
    :param z: Coordenada z do end-effector
    :return: Ângulo da junta rotacional (theta1), deslocamento radial (R), deslocamento vertical (z)
    """
    R = math.sqrt(x**2 + y**2)  # Deslocamento radial
    theta1 = math.atan2(y, -x)  # Ângulo da junta rotacional
    return theta1, R, z

def main():
    # Valores de teste
    x, y, z = 0.26, -0.15, 0.5
    angles = inverse_kinematics(x, y, z)
    print(f"Joint values: θ1={math.degrees(angles[0])}°, R={angles[1]}, z={angles[2]}")
    
if __name__ == '__main__':
    main()
