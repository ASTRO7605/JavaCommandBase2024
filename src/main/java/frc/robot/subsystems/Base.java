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

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.DriveConstant;
import frc.robot.utils.SwerveUtils;

public class Base extends SubsystemBase {
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    // important to follow order
    private final ModuleSwerve m_kFrontRightModule = new ModuleSwerve(DriveConstant.kFrontRightTurningID,
            DriveConstant.kFrontRightDrivingID, DriveConstant.kFrontRightCANcoderID);
    private final ModuleSwerve m_kFrontLeftModule = new ModuleSwerve(DriveConstant.kFrontLeftTurningID,
            DriveConstant.kFrontLeftDrivingID, DriveConstant.kFrontLeftCANcoderID);
    private final ModuleSwerve m_kRearLeftModule = new ModuleSwerve(DriveConstant.kRearLeftTurningID,
            DriveConstant.kRearLeftDrivingID, DriveConstant.kRearLeftCANcoderID);
    private final ModuleSwerve m_kRearRightModule = new ModuleSwerve(DriveConstant.kRearRightTurningID,
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
            new SwerveModulePosition[] { m_kFrontRightModule.getPosition(), m_kFrontLeftModule.getPosition(),
                    m_kRearLeftModule.getPosition(), m_kRearRightModule.getPosition() },
            new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));

    public Base() {
        m_gyro.reset();

        m_poseEstimator.resetPosition(m_gyro.getRotation2d(),
                new SwerveModulePosition[] { m_kFrontRightModule.getPosition(),
                        m_kFrontLeftModule.getPosition(),
                        m_kRearLeftModule.getPosition(), m_kRearRightModule.getPosition() },
                new Pose2d(new Translation2d(0, 0),
                        new Rotation2d(0)));

        SmartDashboard.putData("Robot Measurement", m_robotField);
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(m_gyro.getRotation2d(),
                new SwerveModulePosition[] { m_kFrontRightModule.getPosition(),
                        m_kFrontLeftModule.getPosition(),
                        m_kRearLeftModule.getPosition(), m_kRearRightModule.getPosition() });
        m_robotField.setRobotPose(m_poseEstimator.getEstimatedPosition());
    }

    public void setIdleMode(IdleMode idleMode) {
        m_kFrontRightModule.setIdleMode(idleMode);
        m_kFrontLeftModule.setIdleMode(idleMode);
        m_kRearLeftModule.setIdleMode(idleMode);
        m_kRearRightModule.setIdleMode(idleMode);
    }

    public void resetEncoders() {
        m_kFrontRightModule.resetEncoders();
        m_kFrontLeftModule.resetEncoders();
        m_kRearLeftModule.resetEncoders();
        m_kRearRightModule.resetEncoders();
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
        m_kFrontRightModule.setDesiredState(swerveModuleStates[0]);
        m_kFrontLeftModule.setDesiredState(swerveModuleStates[1]);
        m_kRearLeftModule.setDesiredState(swerveModuleStates[2]);
        m_kRearRightModule.setDesiredState(swerveModuleStates[3]);
    }

    public void setWheelsFacingForward() {
        m_kFrontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(0)));
        m_kFrontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(0)));
        m_kRearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(0)));
        m_kRearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromRadians(0)));
    }

    public void setWheelsInXFormation() {
        m_kFrontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_kFrontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_kRearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_kRearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public double getRobotHeadingDegrees() {
        return m_gyro.getRotation2d().getDegrees();
    }

    public Pose2d getRobotPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d desiredPose) {
        m_poseEstimator.resetPosition(m_gyro.getRotation2d(),
                new SwerveModulePosition[] { m_kFrontRightModule.getPosition(),
                        m_kFrontLeftModule.getPosition(),
                        m_kRearLeftModule.getPosition(), m_kRearRightModule.getPosition() },
                desiredPose);
    }

    public void switchRobotDrivingMode() {
        m_drivingInFieldRelative = !m_drivingInFieldRelative;
    }

    public void setRobotDrivingMode(boolean fieldRelative) {
        m_drivingInFieldRelative = fieldRelative;
    }

    public void resetGyroOffset() {
        m_gyroOffset = m_gyro.getRotation2d().getRadians();
    }

    public void setGyroOffset(double offsetInRadians) {
        m_gyroOffset = offsetInRadians;
    }
}
