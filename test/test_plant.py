# test/test_plant.py

import pytest
from pwm_cltool.plant import Plant


class TestPwmForceScalar:
    @pytest.fixture
    def plant(self):
        return Plant()

    @pytest.mark.parametrize(
        "pwm_raw, expected_branch",
        [
            # raw inputs are in the same units as the code expects,
            # so that after dividing by 1000
            # we hit the intended branches.
            # E.g. 1,200,000/1000 = 1200 → first cubic piece
            (1_200_000, "low"),
            (1_500_000, "mid"),
            (1_600_000, "high"),
        ],
    )
    def test_piecewise_branches(self, plant, pwm_raw, expected_branch):
        """Ensure each branch returns nonzero or zero as designed."""
        f = plant.pwm_force_scalar(pwm_raw)
        if expected_branch == "mid":
            assert f == pytest.approx(0.0)
        else:
            assert f != pytest.approx(0.0)

    @pytest.mark.parametrize(
        "bad_pwm",
        [
            0,  # too small
            1_000,  # 1_000/1000 = 1, <1100
            1_000_000,  # 1_000_000/1000 = 1000, <1100
            2_000_000,  # 2000, >1900
        ],
    )
    def test_out_of_range(self, plant, bad_pwm):
        with pytest.raises(ValueError):
            plant.pwm_force_scalar(bad_pwm)


class TestPwmForce:
    def test_pwm_force_prints_vector(self, capsys):
        """Feeding a valid full set of 8 PWMs should print a 6-element list."""
        plant = Plant()
        # use all midpoints so that scalar→0, net wrench is zero vector
        zeros = [1_500_000.0] * 8
        plant.pwm_force(zeros)
        captured = capsys.readouterr().out.strip()
        # printed as Python list of length 6
        vec = eval(captured)
        assert isinstance(vec, list) and len(vec) == 6
        assert all(v == pytest.approx(0.0) for v in vec)


class TestThrusterGeometry:
    @pytest.fixture
    def plant(self):
        return Plant()

    def test_eight_thrusters(self, plant):
        assert len(plant.thruster_positions) == 8
        assert len(plant.thruster_directions) == 8
        assert len(plant.thruster_torques) == 8

    def test_torque_computed_by_cross(self, plant):
        # verify one index manually
        i = 4
        pos = plant.thruster_positions[i]
        dir = plant.thruster_directions[i]
        expected = Plant.cross_product(pos, dir)
        assert plant.thruster_torques[i] == pytest.approx(expected)

    def test_wrench_matrix_dimensions_and_transpose(self, plant):
        # wrench_matrix_transposed is 8×6
        Wt = plant.wrench_matrix_transposed
        assert len(Wt) == 8 and all(len(row) == 6 for row in Wt)

        # wrench_matrix is its transpose: 6×8
        W = plant.wrench_matrix
        assert len(W) == 6 and all(len(col) == 8 for col in W)

        # spot-check that W[j][i] == Wt[i][j]
        for i in [0, 7]:
            for j in [0, 5]:
                assert W[j][i] == pytest.approx(Wt[i][j])


class TestMatrixUtils:
    @pytest.mark.parametrize(
        "matrix, expected",
        [
            ([[1, 2, 3], [4, 5, 6]], [[1, 4], [2, 5], [3, 6]]),
            ([[42]], [[42]]),
        ],
    )
    def test_transpose_matrix(self, matrix, expected):
        assert Plant.transpose_matrix(matrix) == expected

    @pytest.mark.parametrize(
        "matrix, vector, expected",
        [
            (
                [[1, 0, -1], [2, 3, 4]],
                [1, 2, 3],
                [1 * 1 + 0 * 2 + -1 * 3, 2 * 1 + 3 * 2 + 4 * 3],
            ),
        ],
    )
    def test_matrix_vector_multiply(self, matrix, vector, expected):
        result = Plant.matrix_vector_multiply(matrix, vector)
        assert result == pytest.approx(expected)

    @pytest.mark.parametrize(
        "a, b, expected",
        [
            ([1, 0, 0], [0, 1, 0], [0, 0, 1]),
            ([0, 1, 2], [3, 4, 5], [-6, 6, -3]),
        ],
    )
    def test_cross_product(self, a, b, expected):
        assert Plant.cross_product(a, b) == pytest.approx(expected)

    def test_cross_product_invalid(self):
        with pytest.raises(IndexError):
            Plant.cross_product([1, 2], [1, 2, 3, 4])
