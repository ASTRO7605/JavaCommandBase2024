package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstant;

public class ModuleSwerve extends SubsystemBase {
    private final CANSparkMax m_turningMotor;
    private final CANSparkMax m_drivingMotor;

    private final CANcoder m_turningCANcoder;
    private final RelativeEncoder m_turningSparkMaxEncoder;
    private final RelativeEncoder m_drivingEncoder;

    private final SparkPIDController m_turningPIDController;
    private final SparkPIDController m_drivingPIDController;

    private boolean hasEncoderBeenSeeded = false;

    public ModuleSwerve(int turningMotorID, int drivingMotorID, int CANcoderID) {
        m_turningMotor = new CANSparkMax(turningMotorID, CANSparkLowLevel.MotorType.kBrushless);
        m_drivingMotor = new CANSparkMax(drivingMotorID, CANSparkLowLevel.MotorType.kBrushless);

        m_turningCANcoder = new CANcoder(CANcoderID);
        m_turningSparkMaxEncoder = m_turningMotor.getEncoder();
        m_drivingEncoder = m_drivingMotor.getEncoder();

        m_turningPIDController = m_turningMotor.getPIDController();
        m_drivingPIDController = m_drivingMotor.getPIDController();

        m_turningMotor.restoreFactoryDefaults();
        m_drivingMotor.restoreFactoryDefaults();

        m_turningMotor.setCANTimeout(50);
        m_drivingMotor.setCANTimeout(50);

        m_turningMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 47);
        m_drivingMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 57);

        m_turningCANcoder.getPosition().setUpdateFrequency(20);
        m_turningCANcoder.optimizeBusUtilization();

        m_turningMotor.setInverted(true);
        m_drivingMotor.setInverted(true);

        m_turningMotor.setIdleMode(IdleMode.kBrake);
        m_drivingMotor.setIdleMode(IdleMode.kBrake);

        m_drivingEncoder
                .setPositionConversionFactor(DriveConstant.kWheelCirconf / DriveConstant.kDrivingGearRatio); //motor rotations to meters

        m_drivingEncoder.setVelocityConversionFactor(
                (DriveConstant.kWheelCirconf / DriveConstant.kDrivingGearRatio) / 60); // m/s

        m_turningSparkMaxEncoder.setPosition(m_turningCANcoder.getAbsolutePosition().getValue() * (2 * Math.PI));
        m_turningSparkMaxEncoder.setPositionConversionFactor((2 * Math.PI) / DriveConstant.kTurningGearRatio); // motor rotations to radians
        m_turningSparkMaxEncoder
                .setVelocityConversionFactor(((2 * Math.PI) / DriveConstant.kTurningGearRatio) / 60); // radians/s

        m_turningPIDController.setPositionPIDWrappingEnabled(true); // PID can go through 0 to get to setpoint
        m_turningPIDController.setPositionPIDWrappingMinInput(0);
        m_turningPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);

        m_drivingPIDController.setP(DriveConstant.kPDriving);
        m_drivingPIDController.setI(DriveConstant.kIDriving);
        m_drivingPIDController.setD(DriveConstant.kDDriving);
        m_drivingPIDController.setFF(DriveConstant.kFFDriving);
        m_drivingPIDController.setOutputRange(DriveConstant.kDrivingMinInput, DriveConstant.kDrivingMaxInput);

        m_turningPIDController.setP(DriveConstant.kPTurning);
        m_turningPIDController.setI(DriveConstant.kITurning);
        m_turningPIDController.setD(DriveConstant.kDTurning);
        m_turningPIDController.setFF(DriveConstant.kFFTurning);
        m_turningPIDController.setOutputRange(DriveConstant.kTurningMinInput, DriveConstant.kTurningMaxInput);

        m_drivingMotor.setSmartCurrentLimit((int) DriveConstant.kCurrentLimit);
        m_turningMotor.setSmartCurrentLimit((int) DriveConstant.kCurrentLimit);

        m_drivingEncoder.setPosition(0);

        // m_drivingMotor.burnFlash();
        // m_turningMotor.burnFlash();
    }

    @Override
    public void periodic() {
        if (!hasEncoderBeenSeeded) {
            var absoluteEncoderPose = m_turningCANcoder.getPosition().waitForUpdate(1);
            if (absoluteEncoderPose.getStatus().isOK()) {
                if (m_turningSparkMaxEncoder.setPosition(
                        m_turningCANcoder.getAbsolutePosition().getValue() * (2 * Math.PI)) == REVLibError.kOk) {
                    hasEncoderBeenSeeded = true;
                }
            }
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(),
                new Rotation2d(m_turningSparkMaxEncoder.getPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_drivingEncoder.getPosition(),
                new Rotation2d(m_turningSparkMaxEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_turningSparkMaxEncoder.getPosition()));

        m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond,
                CANSparkMax.ControlType.kVelocity);
        m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(),
                CANSparkMax.ControlType.kPosition);
    }

    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }

    public void setIdleMode(IdleMode idleMode) {
        m_drivingMotor.setIdleMode(idleMode);
        m_turningMotor.setIdleMode(idleMode);
    }

    public double getCANCoderAbsolutePosition() {
        return m_turningCANcoder.getAbsolutePosition().getValue() * 2 * Math.PI;
    }

    public double getCANCoderPosition() {
        return m_turningCANcoder.getPosition().getValue() * 2 * Math.PI;
    }

    public double getTurningSparkMaxPosition() {
        return m_turningSparkMaxEncoder.getPosition();
    }

    public double getDrivingPosition() {
        return m_drivingEncoder.getPosition();
    }

    public double getDrivingVelocity() {
        return m_drivingEncoder.getVelocity();
    }
}