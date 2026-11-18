# GNSS velocity approximation

Finite differences of ground-truth antenna position in world ENU, sampled at
20 Hz with configurable Gaussian velocity noise. Outputs linear velocity in m/s;
angular velocity is unavailable (large covariance). Preserves simulation time.
This does not model ZED-F9P Doppler velocity, RTK status, outages or receiver latency.
