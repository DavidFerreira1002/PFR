import numpy as np

class KalmanFilter3D:
    def __init__(self, process_variance, measurement_variance):
        self.dt = dt  # Time step
        self.process_variance = process_variance  # Process variance (Q)
        self.measurement_variance = measurement_variance  # Measurement variance (R)
        self.posteri_estimate = np.zeros((3, 1))  # Posteriori estimate (x, y, z)
        self.posteri_error_estimate = np.eye(3)  # Posteriori error estimate
        self.priori_estimate = np.zeros((3, 1))  # Priori estimate (x, y, z)
        self.priori_error_estimate = np.eye(3)  # Priori error estimate

    def update(self, measurement,dt):
        # Prediction update
        self.priori_estimate = self.posteri_estimate
        self.priori_error_estimate = self.posteri_error_estimate + dt * self.process_variance

        # Measurement update
        blending_factor = np.linalg.inv(self.priori_error_estimate + self.measurement_variance * np.eye(3))
        kalman_gain = np.dot(self.priori_error_estimate, blending_factor)
        innovation = measurement - self.priori_estimate
        self.posteri_estimate = self.priori_estimate + np.dot(kalman_gain, innovation)
        self.posteri_error_estimate = np.dot(np.eye(3) - kalman_gain, self.priori_error_estimate)

        return self.posteri_estimate

if __name__ == "__main__":
    # Example usage
    
    process_variance = 0.01 * np.eye(3)  # Process variance (Q)
    measurement_variance = 0.01  # Measurement variance (R)

    kf = KalmanFilter3D(process_variance, measurement_variance)

    # Simulate tracking
    time = 0.0
    dt = 1.0  # Time step (1 second)
    while time < 10.0:  # Simulate for 10 seconds
        # Simulate measurement (position of the person)
        measurement = np.random.normal(loc=5.0, scale=1.0, size=(3, 1))  # Simulate a 3D measurement around (5, 5, 5) with some noise
        # Update the Kalman filter with the measurement
        estimated_position = kf.update(measurement)
        print(f"Time: {time}, Measured position: {measurement.flatten()}, Estimated position: {estimated_position.flatten()}")
        time += dt
