# tests/test_plant.py

import pytest
from pwm_cltool.plant import Plant

def test_pwm_force_scalar():
    plant = Plant()
    # Test within valid range
    assert plant.pwm_force_scalar(1200) != 0
    assert plant.pwm_force_scalar(1500) == 0
    assert plant.pwm_force_scalar(1600) != 0
    # Test out of range
    with pytest.raises(ValueError):
        plant.pwm_force_scalar(1000)
