package frc.robot.subsystems;

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
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.DriveConstant;
import frc.robot.utils.SwerveUtils;

public class Base extends SubsystemBase {
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // important to follow order
    private final ModuleSwerve m_frontRightModule = new ModuleSwerve(DriveConstant.kFrontRightTurningID,
            DriveConstant.kFrontRightDrivingID, DriveConstant.kFrontRightCANcoderID);
    private final ModuleSwerve m_frontLeftModule = new ModuleSwerve(DriveConstant.kFrontLeftTurningID,
            DriveConstant.kFrontLeftDrivingID, DriveConstant.kFrontLeftCANcoderID);
    private final ModuleSwerve m_rearLeftModule = new ModuleSwerve(DriveConstant.kRearLeftTurningID,
            DriveConstant.kRearLeftDrivingID, DriveConstant.kRearLeftCANcoderID);
    private final ModuleSwerve m_rearRightModule = new ModuleSwerve(DriveConstant.kRearRightTurningID,
            DriveConstant.kRearRightDrivingID, DriveConstant.kRearRightCANcoderID);

    private final Field2d m_robotField = new Field2d();

    private double m_currentRotationSpeed;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;
    private boolean m_drivingInFieldRelative = true;

    /** radians */
    private double m_gyroOffset = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstant.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstant.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstant.kDriveKinematics, m_gyro.getRotation2d(),
            new SwerveModulePosition[] { m_frontRightModule.getPosition(), m_frontLeftModule.getPosition(),
                    m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() },
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

    public Base() {
        m_gyro.reset();

        m_poseEstimator.resetPosition(m_gyro.getRotation2d(),
                new SwerveModulePosition[] { m_frontRightModule.getPosition(),
                        m_frontLeftModule.getPosition(),
                        m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() },
                new Pose2d(new Translation2d(0, 0),
                        new Rotation2d(0)));

        AutoBuilder.configureHolonomic(
                this::getRobotPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelativeChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(DriveConstant.kPAutoMovementController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(DriveConstant.kPAutoMovementController, 0.0, 0.0), // Rotation PID constants
                        DriveConstant.kMaxAutoSpeed, // Max module speed, in m/s
                        DriveConstant.kChassisRadius, // Drive base radius in meters. Distance from robot center to furthest module.
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

        SmartDashboard.putData("Robot Measurement", m_robotField);
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(m_gyro.getRotation2d(),
                new SwerveModulePosition[] { m_frontRightModule.getPosition(),
                        m_frontLeftModule.getPosition(),
                        m_rearLeftModule.getPosition(), m_rearRightModule.getPosition() });
        m_robotField.setRobotPose(m_poseEstimator.getEstimatedPosition());
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
     *            Speed of the robot in the x direction (forward)(m/s).
     * @param ySpeed
     *            Speed of the robot in the y direction (sideways)(m/s).
     * @param rotationSpeed
     *            Angular rate of the robot(radians/s).
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
                directionSlewRate = Math.abs(DriveConstant.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
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

        double xSpeedDelivered = xSpeedCommanded * DriveConstant.kMaxTeleopSpeed;
        double ySpeedDelivered = ySpeedCommanded * DriveConstant.kMaxTeleopSpeed;
        double rotationSpeedDelivered = m_currentRotationSpeed * DriveConstant.kMaxTeleopAngularSpeed;

        var swerveModuleStates = DriveConstant.kDriveKinematics
                .toSwerveModuleStates(m_drivingInFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
                        rotationSpeedDelivered, Rotation2d.fromRadians(m_gyro.getRotation2d().getRadians() - m_gyroOffset))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered,
                                rotationSpeedDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstant.kMaxTeleopSpeed);
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

    public double getRobotHeadingDegrees() {
        return m_gyro.getRotation2d().getDegrees();
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

    /** resets offset to current gyro value */
    public void resetGyroOffset() {
        m_gyroOffset = m_gyro.getRotation2d().getRadians();
    }

    /** resets offset to value passed in */
    public void resetGyroOffset(double offsetInRadians) {
        m_gyroOffset = offsetInRadians;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstant.kDriveKinematics.toChassisSpeeds(m_frontRightModule.getState(), m_frontLeftModule.getState(),
                m_rearLeftModule.getState(), m_rearRightModule.getState());
    }

    public void driveRobotRelativeChassisSpeeds(ChassisSpeeds speeds) {
        var swerveModuleStates = DriveConstant.kDriveKinematics.toSwerveModuleStates(speeds);

        m_frontRightModule.setDesiredState(swerveModuleStates[0]);
        m_frontLeftModule.setDesiredState(swerveModuleStates[1]);
        m_rearLeftModule.setDesiredState(swerveModuleStates[2]);
        m_rearRightModule.setDesiredState(swerveModuleStates[3]);
    }
}
