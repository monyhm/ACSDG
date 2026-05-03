import numpy as np


class KalmanTracker:
    def __init__(self, threat_id):
        self.threat_id = threat_id
        self.dt = 0.1  # 100ms

        # State vector: [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)
        self.hits = 0
        # State transition matrix
        self.F = np.eye(6)
        for i in range(3):
            self.F[i, i + 3] = self.dt

        # Measurement matrix (we only measure x, y, z)
        self.H = np.zeros((3, 6))
        self.H[0:3, 0:3] = np.eye(3)

        # Covariance matrices
        # Position uncertainty: moderate
        # Velocity uncertainty: low (we don't trust sudden velocity changes)
        self.P = np.diag([2.0, 2.0, 10.0, 0.5, 0.5, 0.5])

        # Measurement noise: higher = trust measurements less, smoother output
        self.R = np.eye(3) * 25.0

        # Process noise: very low velocity noise = velocity changes slowly
        self.Q = np.diag([0.01, 0.01, 0.5, 0.001, 0.001, 0.001])

        self.confidence = 0.95
        self._predicted = False

    def predict(self):
        """Advance time by dt — call once per 100ms tick, not per measurement."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self._predicted = True

    def update(self, measurement):
        """Incorporate one sensor measurement — never predicts."""
        z = np.array(measurement)
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P
        self.hits += 1

    def reset_predict_flag(self):
        """Call once per tick after all tower readings processed."""
        self._predicted = False

    def get_state(self):
        return self.x.tolist()
