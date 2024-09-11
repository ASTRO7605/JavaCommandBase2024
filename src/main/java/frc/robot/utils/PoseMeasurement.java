package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose3d;

public class PoseMeasurement {
    Pose3d m_pose;
    /** seconds */
    double m_timestamp;
    /** meters */
    double m_distance;
    double m_ambiguity;

    /** An AprilTag measurement result */
    public PoseMeasurement(Pose3d pose, double timestampSeconds, double distanceMeters, double ambiguity) {
        m_pose = pose;
        m_timestamp = timestampSeconds;
        m_distance = distanceMeters;
        m_ambiguity = ambiguity;
    }

    public Pose3d getPose() {
        return m_pose;
    }

    /** seconds */
    public double getTimestamp() {
        return m_timestamp;
    }

    /** meters */
    public double getDistance() {
        return m_distance;
    }

    public double getAmbiguity() {
        return m_ambiguity;
    }

    public void setAmbiguity(double ambiguity) {
        m_ambiguity = ambiguity;
    }
}
