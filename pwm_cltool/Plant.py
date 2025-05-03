import math

class Plant:
    def __init__(self):
        # Thruster positions
        self.thruster_positions = [
            [0.2535, -0.2035, 0.042],
            [0.2535, 0.2035, 0.042],
            [-0.2545, -0.2035, 0.042],
            [-0.2545, 0.2035, 0.042],
            [0.1670, -0.1375, -0.049],
            [0.1670, 0.1375, -0.049],
            [-0.1975, -0.1165, -0.049],
            [-0.1975, 0.1165, -0.049],
        ]

        # Thruster directions
        sin45 = math.sin(math.pi / 4)
        self.thruster_directions = [
            [0, 0, 1],
            [0, 0, -1],
            [0, 0, 1],
            [0, 0, -1],
            [-sin45, -sin45, 0],
            [sin45, -sin45, 0],
            [-sin45, sin45, 0],
            [sin45, sin45, 0],
        ]

        # Thruster torques
        self.thruster_torques = [
            self.cross_product(self.thruster_positions[i], self.thruster_directions[i])
            for i in range(8)
        ]

        # Compute wrench matrix (6x8)
        self.wrench_matrix_transposed = [[0] * 6 for _ in range(8)]
        for i in range(8):
            self.wrench_matrix_transposed[i][0:3] = self.thruster_directions[i]
            self.wrench_matrix_transposed[i][3:6] = self.thruster_torques[i]

        # Transpose to get wrench matrix (6x8)
        self.wrench_matrix = self.transpose_matrix(self.wrench_matrix_transposed)

    def pwm_force_scalar(self, x):
        x = x / 1000
        if 1100 <= x < 1460:
            force = (
                (-1.24422882971549e-8) * x**3
                + (4.02057100632393e-5) * x**2
                - 0.0348619861030835 * x
                + 3.90671429105423
            )
        elif 1460 <= x <= 1540:
            force = 0
        elif 1540 < x <= 1900:
            force = (
                (-1.64293565374284e-8) * x**3
                + (9.45962838560648e-5) * x**2
                - 0.170812079190679 * x
                + 98.7232373648272
            )
        else:
            raise ValueError("PWM value out of valid range (1100-1900)")
        return force

    def pwm_force(self, pwm_set):
        thruster_forces = [
            self.pwm_force_scalar(pwm_set[i]) for i in range(len(pwm_set))
        ]
        force = self.matrix_vector_multiply(self.wrench_matrix, thruster_forces)
        print(force)

    @staticmethod
    def transpose_matrix(matrix):
        """Transposes a 2D list (matrix)."""
        return [[row[i] for row in matrix] for i in range(len(matrix[0]))]

    @staticmethod
    def matrix_vector_multiply(matrix, vector):
        """Multiplies a matrix (list of lists) by a vector (list)."""
        return [
            sum(matrix[i][j] * vector[j] for j in range(len(vector)))
            for i in range(len(matrix))
        ]

    @staticmethod
    def cross_product(a, b):
        """Computes the cross product of two 3D vectors a and b."""
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]