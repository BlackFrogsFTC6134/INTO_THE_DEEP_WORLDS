package org.firstinspires.ftc.teamcode.subsystem.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.ejml.simple.SimpleMatrix;

public class KalmanFilter3D {
    // Our state is [x, y, theta]^T.
    private SimpleMatrix x;
    // Covariance (3x3).
    private SimpleMatrix P;
    // Process noise covariance (3x3).
    private SimpleMatrix Q;
    // Measurement noise covariance (3x3).
    private SimpleMatrix R;

    /**
     * @param initialPose The starting pose.
     * @param processNoise Scalar noise for process covariance.
     * @param measurementNoise Scalar noise for measurement covariance.
     */
    public KalmanFilter3D(Pose2d initialPose, double[] processNoiseDiagonal, double[] measurementNoiseDiagonal) {
        // Initialize state vector.
        x = new SimpleMatrix(new double[][] {
                {initialPose.getX()},
                {initialPose.getY()},
                {initialPose.getHeading()}
        });
        // Initialize covariance (start with small uncertainty).
        P = SimpleMatrix.identity(3).scale(1.0);
        // Process noise; initialize as diagonal matrix.
        Q = new SimpleMatrix(3,3);
        for (int i = 0; i < 3; i++) {
            Q.set(i, i, processNoiseDiagonal[i]);
        }
        // Measurement noise; initialize as diagonal matrix.
        R = new SimpleMatrix(3,3);
        for (int i = 0; i < 3; i++) {
            R.set(i, i, measurementNoiseDiagonal[i]);
        }
    }

    /**
     * Prediction step. Here we assume a simple constant state model.
     */
    public void predict() {
        // x = x (no control input)
        // Update covariance: P = P + Q.
        P = P.plus(Q);
    }

    /**
     * Update the filter with a new measurement.
     * @param measurement The measured pose.
     */
    public void update(Pose2d measurement) {
        SimpleMatrix z = new SimpleMatrix(new double[][] {
                {measurement.getX()},
                {measurement.getY()},
                {measurement.getHeading()}
        });
        // Our measurement matrix H is identity.
        SimpleMatrix H = SimpleMatrix.identity(3);
        // Innovation: y = z - H*x.
        SimpleMatrix y = z.minus(H.mult(x));
        // Innovation covariance: S = H*P*H^T + R.
        SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);
        // Kalman gain: K = P*H^T * S^-1.
        SimpleMatrix K = P.mult(H.transpose()).mult(S.invert());
        // Updated state estimate.
        x = x.plus(K.mult(y));
        // Updated covariance.
        SimpleMatrix I = SimpleMatrix.identity(3);
        P = (I.minus(K.mult(H))).mult(P);
    }

    public Pose2d getState() {
        return new Pose2d(x.get(0, 0), x.get(1, 0), x.get(2, 0));
    }
}
