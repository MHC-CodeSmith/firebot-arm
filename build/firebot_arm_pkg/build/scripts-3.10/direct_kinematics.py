import math

def direct_kinematics(theta1, z, R):
    """
    Calcula a posição do end-effector com base nos ângulos e deslocamentos das juntas.

    :param theta1: Ângulo da junta rotacional em radianos
    :param z: Deslocamento prismatic na direção vertical
    :param R: Deslocamento prismatic na direção radial
    :return: Posição cartesiana do end-effector (x, y, z)
    """
    x = -R * math.sin(theta1)  # Coordenada x
    y = R * math.cos(theta1)   # Coordenada y
    return x, y, z

def main():
    # Valores de teste
    theta1 = math.radians(30)  # Ângulo em graus convertido para radianos
    z = 0.5
    R = 0.3
    pos = direct_kinematics(theta1, z, R)
    print(f"End-effector position: x={pos[0]}, y={pos[1]}, z={pos[2]}")

if __name__ == '__main__':
    main()
