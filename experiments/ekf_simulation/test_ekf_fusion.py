"""Checks synthetic EKF behavior, not real sensor accuracy."""
import numpy as np
import pytest
from ekf_fusion import PoseEKF, simulate

def test_covariance_symmetric_psd():
    f=PoseEKF()
    for j in range(100):
        f.predict(0.1,0.0,0.02)
        if j%10==0: f.update([0.05*j,0.0])
        assert np.allclose(f.P,f.P.T,atol=1e-10)
        assert np.min(np.linalg.eigvalsh(f.P)) >= -1e-9

def test_dropout_and_invalid_inputs():
    f=PoseEKF()
    assert f.update(None) is False
    with pytest.raises(ValueError): f.update([np.nan,0])
    with pytest.raises(ValueError): f.predict(np.nan,0,0)

def test_simulation_is_reproducible():
    _, a, i=simulate(seed=42,seconds=10)
    _, b, j=simulate(seed=42,seconds=10)
    assert a==b and i==j

def test_fusion_improves_on_imu_drift_in_example():
    _,metrics,_=simulate()
    assert metrics['ekf']['rmse_m'] < metrics['imu_only']['rmse_m']
    assert np.isfinite(metrics['ekf']['final_error_m'])
