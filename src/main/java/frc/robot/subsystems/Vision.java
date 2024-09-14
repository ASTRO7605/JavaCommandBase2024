package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.PoseMeasurement;

public class Vision extends SubsystemBase {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_photonPoseEstimator;

    private PhotonPipelineResult m_latestResult;
    private MultiTargetPNPResult m_latestMultiResult;
    private PhotonTrackedTarget m_latestSingleResult;

    private boolean m_isLatestSingleResultValid;
    private boolean m_isLatestMultiResultValid;
    private double m_targetDistance;
    private double m_targetAmbiguity;
    private int m_lastAprilTagSeen;

    public Vision(String tableName, Transform3d robotToCameraTransform) {
        m_camera = new PhotonCamera(tableName);
        m_photonPoseEstimator = new PhotonPoseEstimator(
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_camera, robotToCameraTransform);

        m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        m_isLatestMultiResultValid = false;
        m_isLatestSingleResultValid = false;

        m_latestResult = m_camera.getLatestResult();
        if (m_latestResult.hasTargets()) {
            m_lastAprilTagSeen = m_latestResult.getBestTarget().getFiducialId();
            m_latestMultiResult = m_latestResult.getMultiTagResult();
            m_latestSingleResult = m_latestResult.getBestTarget();
            if (m_latestMultiResult.estimatedPose.isPresent) {
                if (m_latestMultiResult.estimatedPose.ambiguity < VisionConstants.kAmbiguityThreshold) {
                    m_isLatestMultiResultValid = true;
                }
            } else if (m_latestSingleResult
                    .getPoseAmbiguity() < VisionConstants.kAmbiguityThreshold) {
                m_isLatestSingleResultValid = true;
            }
        } else {
            m_lastAprilTagSeen = 0;
        }
    }

    public boolean seesValidTarget() {
        return (m_isLatestMultiResultValid || m_isLatestSingleResultValid);
    }

    public int getAprilTagIDInView() {
        return m_lastAprilTagSeen;
    }

    public double getAprilTagDistanceMeters(double xDistance, double yDistance, double zDistance) {
        return Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2) + Math.pow(zDistance, 2));
    }

    public Optional<PoseMeasurement> getRobotPoseEstimate() {
        var poseEstimate = m_photonPoseEstimator.update();
        if (!poseEstimate.isPresent()) {
            return Optional.empty();
        }

        if (m_isLatestMultiResultValid) {
            m_targetDistance = getAprilTagDistanceMeters(m_latestMultiResult.estimatedPose.best.getX(),
                    m_latestMultiResult.estimatedPose.best.getY(),
                    m_latestMultiResult.estimatedPose.best.getZ());
            m_targetAmbiguity = m_latestMultiResult.estimatedPose.ambiguity;
        } else if (m_isLatestSingleResultValid) {
            m_targetDistance = getAprilTagDistanceMeters(m_latestSingleResult.getBestCameraToTarget().getX(),
                    m_latestSingleResult.getBestCameraToTarget().getY(),
                    m_latestSingleResult.getBestCameraToTarget().getZ());
            m_targetAmbiguity = m_latestSingleResult.getPoseAmbiguity();
        } else {
            return Optional.empty();
        }
        PoseMeasurement returnVal = new PoseMeasurement(poseEstimate.get().estimatedPose,
                poseEstimate.get().timestampSeconds, m_targetDistance, m_targetAmbiguity);

        if (Double.isNaN(returnVal.getPose().getX()) || Double.isNaN(returnVal.getPose().getY())
                || Double.isNaN(returnVal.getPose().getZ())
                || Double.isNaN(returnVal.getPose().getRotation().toRotation2d().getDegrees())) {
            return Optional.empty();
        }
        return Optional.of(returnVal);
    }
}
