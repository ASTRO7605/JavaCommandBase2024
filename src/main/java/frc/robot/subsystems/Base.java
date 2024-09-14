package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PoseEstimationConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.PoseMeasurement;
import frc.robot.utils.SwerveUtils;

public class Base extends SubsystemBase {

    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // important to follow order
    private final ModuleSwerve m_frontRightModule = new ModuleSwerve(
            DriveConstants.kFrontRightTurningID,
            DriveConstants.kFrontRightDrivingID, DriveConstants.kFrontRightCANcoderID);
    private final ModuleSwerve m_frontLeftModule = new ModuleSwerve(
            DriveConstants.kFrontLeftTurningID,
            DriveConstants.kFrontLeftDrivingID, DriveConstants.kFrontLeftCANcoderID);
    private final ModuleSwerve m_rearLeftModule = new ModuleSwerve(DriveConstants.kRearLeftTurningID,
            DriveConstants.kRearLeftDrivingID, DriveConstants.kRearLeftCANcoderID);
    private final ModuleSwerve m_rearRightModule = new ModuleSwerve(
            DriveConstants.kRearRightTurningID,
            DriveConstants.kRearRightDrivingID, DriveConstants.kRearRightCANcoderID);

    private final PIDController m_pidControllerThetaSpeaker = new PIDController(
            DriveConstants.kPThetaRobot, DriveConstants.kIThetaRobot,
            DriveConstants.kDThetaRobot);

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
            new SwerveModulePosition[] { m_frontRightModule.getPosition(),
                    m_frontLeftModule.getPosition(),
                    m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() },
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
            new Matrix<>(Nat.N3(), Nat.N1(), PoseEstimationConstants.kStateStdDevs),
            new Matrix<>(Nat.N3(), Nat.N1(), PoseEstimationConstants.kVisionStdDevsDefault));

    private DriverStation.Alliance m_allianceColor = DriverStation.Alliance.Red;

    private final Field2d m_visionFieldLeft = new Field2d();
    private final Field2d m_visionFieldRight = new Field2d();
    private final Field2d m_robotField = new Field2d();

    private final Vision m_visionLeft = new Vision(VisionConstants.kTableNameLeft,
            VisionConstants.kLeftCameraTransform);
    private final Vision m_visionRight = new Vision(VisionConstants.kTableNameRight,
            VisionConstants.kRightCameraTransform);

    private Pose2d m_lastPoseEstimate = new Pose2d();
    private Translation2d m_currentColorSpeakerPose;

    private double m_currentRotationSpeed;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;
    private boolean m_drivingInFieldRelative = true;
    private boolean m_isRotationBeingControlled = false;
    /** radians */
    private double m_gyroOffset = 0.0;

    public Base() {
        m_gyro.reset();

        m_pidControllerThetaSpeaker.enableContinuousInput(-1.0, 1.0);

        AutoBuilder.configureHolonomic(
                this::getRobotPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelativeChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(DriveConstants.kPAutoMovementController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(DriveConstants.kPAutoMovementController, 0.0, 0.0), // Rotation PID constants
                        DriveConstants.kMaxAutoSpeed, // Max module speed, in m/s
                        DriveConstants.kChassisRadius, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        SmartDashboard.putData("Vision Measurement Right", m_visionFieldRight);
        SmartDashboard.putData("Vision Measurement Left", m_visionFieldLeft);
        SmartDashboard.putData("Robot Measurement", m_robotField);

        m_poseEstimator.resetPosition(m_gyro.getRotation2d(),
                new SwerveModulePosition[] { m_frontRightModule.getPosition(),
                        m_frontLeftModule.getPosition(),
                        m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() },
                new Pose2d(new Translation2d(0, 0),
                        new Rotation2d(0)));

    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isRotationBeingControlled", m_isRotationBeingControlled);
        // NaN error checking (not sure if still useful)
        if (Double.isNaN(getRobotPose().getX()) || Double.isNaN(getRobotPose().getY())
                || Double.isNaN(getRobotPose().getRotation().getDegrees())) {
            resetOdometry(m_lastPoseEstimate);
        }
        m_lastPoseEstimate = getRobotPose();

        if (DriverStation.getAlliance().isPresent()) {
            m_allianceColor = DriverStation.getAlliance().get();
        }

        if (m_allianceColor == DriverStation.Alliance.Blue) {
            m_currentColorSpeakerPose = PoseEstimationConstants.kBlueSpeakerPoseMeters;
        } else if (m_allianceColor == DriverStation.Alliance.Red) {
            m_currentColorSpeakerPose = PoseEstimationConstants.kRedSpeakerPoseMeters;
        }

        SmartDashboard.putNumber("Left Camera AprilTag ID", getCameraAprilTagID(m_visionLeft));
        SmartDashboard.putNumber("Right Camera AprilTag ID", getCameraAprilTagID(m_visionRight));

        SmartDashboard.putNumber("Distance to Speaker Meters", getDistanceToSpeaker());
        SmartDashboard.putNumber("Desired Rotation to Speaker Degrees", Math.toDegrees(getDesiredRotationToSpeaker()));
        SmartDashboard.putNumber("Rotation PID Error Speaker", getRotationPIDError());

        SmartDashboard.putNumber("Gyro Angle Degrees", getRobotHeading().getDegrees());
        SmartDashboard.putNumber("Robot Estimated Pose X", getRobotPose().getX());
        SmartDashboard.putNumber("Robot Estimated Pose Y", getRobotPose().getY());
        SmartDashboard.putNumber("Robot Estimated Orientation Degrees", getRobotPose().getRotation().getDegrees());

        m_poseEstimator.update(m_gyro.getRotation2d(),
                new SwerveModulePosition[] { m_frontRightModule.getPosition(),
                        m_frontLeftModule.getPosition(),
                        m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() });

        setRobotPoseVisionEstimate(m_visionLeft, m_visionFieldLeft);
        setRobotPoseVisionEstimate(m_visionRight, m_visionFieldRight);

        m_robotField.setRobotPose(m_poseEstimator.getEstimatedPosition());
        SmartDashboard.putNumber("Robot Field X Speed", getFieldRelativeSpeeds().vxMetersPerSecond);
        SmartDashboard.putNumber("Robot Field Y Speed", getFieldRelativeSpeeds().vyMetersPerSecond);
    }

    public void setIdleMode(IdleMode idleMode) {
        m_frontRightModule.setIdleMode(idleMode);
        m_frontLeftModule.setIdleMode(idleMode);
        m_rearLeftModule.setIdleMode(idleMode);
        m_rearRightModule.setIdleMode(idleMode);
    }

    public void resetEncoders() {
        m_frontRightModule.resetEncoders();
        m_frontLeftModule.resetEncoders();
        m_rearLeftModule.resetEncoders();
        m_rearRightModule.resetEncoders();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed
     *            Speed of the robot in the x direction (forward)([-1 , 1]).
     * @param ySpeed
     *            Speed of the robot in the y direction (sideways)([-1 , 1]).
     * @param rotationSpeed
     *            Angular rate of the robot([-1 , 1]).
     * 
     * @param rateLimiting
     *            Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rotationSpeed,
            boolean rateLimiting) {
        // code vient de REV MAXSwerve
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimiting) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math
                        .abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir,
                    m_currentTranslationDir);

            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils
                            .WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir,
                        inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }

            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotationSpeed = m_rotLimiter.calculate(rotationSpeed);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotationSpeed = rotationSpeed;
        }

        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxTeleopSpeed;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxTeleopSpeed;
        double rotationSpeedDelivered = m_currentRotationSpeed
                * DriveConstants.kMaxTeleopAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics
                .toSwerveModuleStates(m_drivingInFieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
                                rotationSpeedDelivered,
                                Rotation2d.fromRadians(
                                        m_gyro.getRotation2d().getRadians() - m_gyroOffset))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered,
                                rotationSpeedDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
                DriveConstants.kMaxTeleopSpeed);
        //same order as kinematics
        m_frontRightModule.setDesiredState(swerveModuleStates[0]);
        m_frontLeftModule.setDesiredState(swerveModuleStates[1]);
        m_rearLeftModule.setDesiredState(swerveModuleStates[2]);
        m_rearRightModule.setDesiredState(swerveModuleStates[3]);
    }

    public void setWheelsFacingForward() {
        m_frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(0)));
        m_frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(0)));
        m_rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(0)));
        m_rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(0)));
    }

    public void setWheelsInXFormation() {
        m_frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public Rotation2d getRobotHeading() {
        return m_gyro.getRotation2d();
    }

    public Pose2d getRobotPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d desiredPose) {
        m_poseEstimator.resetPosition(m_gyro.getRotation2d(),
                new SwerveModulePosition[] { m_frontRightModule.getPosition(),
                        m_frontLeftModule.getPosition(),
                        m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() },
                desiredPose);
    }

    public void switchRobotDrivingMode() {
        m_drivingInFieldRelative = !m_drivingInFieldRelative;
    }

    public void setRobotDrivingMode(boolean fieldRelative) {
        m_drivingInFieldRelative = fieldRelative;
    }

    /**
     * resets current gyro offset for field oriented driving
     * 
     * @param usePoseEstimator
     *            whether or not to use the rotation value from the pose estimator
     *            to reset. If false, use the current value from the gyro.
     */
    public void resetGyroOffset(boolean usePoseEstimator) {
        if (!usePoseEstimator) {
            m_gyroOffset = m_gyro.getRotation2d().getRadians();
        } else {
            double currentHeading = m_poseEstimator.getEstimatedPosition().getRotation().getRadians();
            double currentGyroAngle = m_gyro.getRotation2d().getRadians();
            double targetHeading = (m_allianceColor == DriverStation.Alliance.Blue) ? 0
                    : Math.toRadians(180);

            double error = targetHeading - currentHeading;

            m_gyroOffset = currentGyroAngle + error;
        }
    }

    /** resets offset to value passed in */
    public void resetGyroOffset(double offsetInRadians) {
        m_gyroOffset = offsetInRadians;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontRightModule.getState(),
                m_frontLeftModule.getState(),
                m_rearLeftModule.getState(), m_rearRightModule.getState());
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        ChassisSpeeds robotRelativeSpeeds = getRobotRelativeSpeeds();

        Rotation2d robotAngle = new Rotation2d(m_gyro.getRotation2d().getRadians() - m_gyroOffset
                - ((m_allianceColor == DriverStation.Alliance.Red) ? Math.toRadians(180) : 0));

        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, robotAngle);

    }

    public void driveRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

        m_frontRightModule.setDesiredState(swerveModuleStates[0]);
        m_frontLeftModule.setDesiredState(swerveModuleStates[1]);
        m_rearLeftModule.setDesiredState(swerveModuleStates[2]);
        m_rearRightModule.setDesiredState(swerveModuleStates[3]);
    }

    public boolean isRotationBeingControlled() {
        return m_isRotationBeingControlled;
    }

    public void setRotationBeingControlled(boolean isRotationBeingControlled) {
        m_isRotationBeingControlled = isRotationBeingControlled;
    }

    /** meters */
    public double getDistanceToSpeaker() {
        Translation2d currentPose = m_poseEstimator.getEstimatedPosition().getTranslation();
        currentPose = currentPose.plus(getProjectedPositionOffset(true));
        var poseToSpeaker = currentPose.minus(m_currentColorSpeakerPose);
        return Math.sqrt(Math.pow(poseToSpeaker.getX(), 2) + Math.pow(poseToSpeaker.getY(), 2));
    }

    /** radians */
    public double getDesiredRotationToSpeaker() {
        Translation2d currentPose = m_poseEstimator.getEstimatedPosition().getTranslation();
        currentPose = currentPose.plus(getProjectedPositionOffset(false));
        var poseToSpeaker = currentPose.minus(m_currentColorSpeakerPose);
        SmartDashboard.putNumber("Pose To Speaker X", poseToSpeaker.getX());
        SmartDashboard.putNumber("Pose To Speaker Y", poseToSpeaker.getY());
        double desiredRotation = Math.atan2(poseToSpeaker.getY(), poseToSpeaker.getX());
        if (desiredRotation < 0) {
            desiredRotation = Math.toRadians(180) + desiredRotation;
        } else {
            desiredRotation = Math.toRadians(-180) + desiredRotation;
        }
        return desiredRotation;
    }

    public double getPIDControlledRotationSpeed(boolean alignToSpeaker) {
        if (alignToSpeaker) {
            return m_pidControllerThetaSpeaker.calculate(
                    (m_poseEstimator.getEstimatedPosition().getRotation().getRadians() / (2 * Math.PI)),
                    (getDesiredRotationToSpeaker() / (2 * Math.PI))); // constraint to [-1, 1]
        } else {
            double sourceAngleTarget;
            if (m_allianceColor == DriverStation.Alliance.Blue) {
                sourceAngleTarget = DriveConstants.kBlueSourceApproachAngle;
            } else if (m_allianceColor == DriverStation.Alliance.Red) {
                sourceAngleTarget = DriveConstants.kRedSourceApproachAngle;
            } else {
                return 0;
            }

            return m_pidControllerThetaSpeaker.calculate(
                    (m_poseEstimator.getEstimatedPosition().getRotation().getRadians() / (2 * Math.PI)),
                    sourceAngleTarget / (2 * Math.PI));

        }
    }

    public boolean isRobotInRangeToShoot() {
        if (getDistanceToSpeaker() <= DriveConstants.kThresholdSpeakerInRangeToShoot) {
            return true;
        }
        return false;
    }

    public boolean isRobotInRangeToStartWheels() {
        if (getDistanceToSpeaker() <= DriveConstants.kThresholdSpeakerInRangeToStartWheels) {
            return true;
        }
        return false;
    }

    public double getRotationPIDError() {
        return m_pidControllerThetaSpeaker.getPositionError();
    }

    public int getCameraAprilTagID(Vision visionObject) {
        return visionObject.getAprilTagIDInView();
    }

    public Optional<Pose2d> getAveragePoseFromCameras() {
        var leftEstimate = m_visionLeft.getRobotPoseEstimate();
        var rightEstimate = m_visionRight.getRobotPoseEstimate();

        if (leftEstimate.isPresent() && rightEstimate.isPresent()) {
            var leftPose = leftEstimate.get().getPose().toPose2d();
            var rightPose = rightEstimate.get().getPose().toPose2d();

            var translation = (leftPose.getTranslation().plus(rightPose.getTranslation())).div(2); // average
            var rotation = (leftPose.getRotation().plus(rightPose.getRotation())).div(2); // average

            return Optional.of(new Pose2d(translation, rotation));
        } else if (leftEstimate.isPresent()) {
            return Optional.of(leftEstimate.get().getPose().toPose2d());
        } else if (rightEstimate.isPresent()) {
            return Optional.of(rightEstimate.get().getPose().toPose2d());
        } else {
            return Optional.empty();
        }
    }

    public boolean isRobotAlignedToShoot() {
        if (Math.abs(m_poseEstimator.getEstimatedPosition().getRotation().getRadians()
                - getDesiredRotationToSpeaker()) <= DriveConstants.kThresholdRobotAngle) {
            return true;
        }
        return false;
    }

    Translation2d getProjectedPositionOffset(boolean usedForDistance) {
        ChassisSpeeds currentRobotSpeeds = getFieldRelativeSpeeds();
        double timeForProjectionInFuture;
        if (usedForDistance) {
            timeForProjectionInFuture = DriveConstants.kTimeForProjectionInFutureDistance;
        } else {
            timeForProjectionInFuture = DriveConstants.kTimeForProjectionInFutureRotation;
        }
        return new Translation2d(currentRobotSpeeds.vxMetersPerSecond * timeForProjectionInFuture,
                currentRobotSpeeds.vyMetersPerSecond * timeForProjectionInFuture);
    }

    private void setRobotPoseVisionEstimate(Vision visionObject, Field2d visionField2dObject) {
        if (!visionObject.seesValidTarget()) {
            // hide robot on dashboard
            visionField2dObject.setRobotPose(100.0, 100.0, new Rotation2d());
            return;
        }

        Optional<PoseMeasurement> estimate = visionObject.getRobotPoseEstimate();
        if (!estimate.isPresent()) {
            return;
        }

        Pose2d measurement2d = estimate.get().getPose().toPose2d();

        if (estimate.get().getAmbiguity() == 0) {
            estimate.get().setAmbiguity(0.001);
        }

        var stdDevs = PoseEstimationConstants.kVisionStdDevsPerAmbiguityPerMeter;

        stdDevs[0] = estimate.get().getAmbiguity() * stdDevs[0] + PoseEstimationConstants.kVisionStdDevsPerMeterBase[0];
        stdDevs[1] = estimate.get().getAmbiguity() * stdDevs[1] + PoseEstimationConstants.kVisionStdDevsPerMeterBase[1];
        stdDevs[2] = estimate.get().getAmbiguity() * stdDevs[2] + PoseEstimationConstants.kVisionStdDevsPerMeterBase[2];

        var dst = estimate.get().getDistance();
        stdDevs[0] *= dst; // scale based on distance
        stdDevs[1] *= dst;
        stdDevs[2] *= dst;

        SmartDashboard.putNumber("Distance AprilTag " + (visionObject.equals(m_visionLeft) ? "Left" : "Right"),
                estimate.get().getDistance());
        SmartDashboard.putNumber("Ambiguity AprilTag " + (visionObject.equals(m_visionLeft) ? "Left" : "Right"),
                estimate.get().getAmbiguity());

        m_poseEstimator.addVisionMeasurement(measurement2d, estimate.get().getTimestamp(),
                new Matrix<N3, N1>(Nat.N3(), Nat.N1(), stdDevs));

        visionField2dObject.setRobotPose(measurement2d);
    }

}
