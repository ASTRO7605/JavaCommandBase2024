package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BarreConstants;
import frc.robot.Constants.FacteursConversion;

public class Barre extends SubsystemBase {
    private final TalonSRX m_moteurPremierJoint;
    private final TalonSRX m_moteurDeuxiemeJoint;

    public Barre() {
        SmartDashboard.putString("test", "in barre constructor");
        m_moteurPremierJoint = new TalonSRX(BarreConstants.PremierJoint.kMoteurID);
        m_moteurDeuxiemeJoint = new TalonSRX(BarreConstants.DeuxiemeJoint.kMoteurID);
        // init can be optimized
        m_moteurPremierJoint.setNeutralMode(NeutralMode.Brake);
        m_moteurDeuxiemeJoint.setNeutralMode(NeutralMode.Brake);

        m_moteurPremierJoint.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
                BarreConstants.kTimeout);

        m_moteurPremierJoint.configFeedbackNotContinuous(true, BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configFeedbackNotContinuous(true, BarreConstants.kTimeout);

        m_moteurPremierJoint.setSensorPhase(true);
        m_moteurDeuxiemeJoint.setSensorPhase(false);

        m_moteurPremierJoint.setInverted(true);
        m_moteurDeuxiemeJoint.setInverted(true);

        seedEncoderPremierJoint();
        seedEncoderDeuxiemeJoint();

        m_moteurPremierJoint.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 101,
                BarreConstants.kTimeout);
        m_moteurPremierJoint.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 101,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 101,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 101,
                BarreConstants.kTimeout);

        m_moteurPremierJoint.configPeakOutputForward(BarreConstants.kPeakOutputForward,
                BarreConstants.kTimeout);
        m_moteurPremierJoint.configPeakOutputReverse(BarreConstants.kPeakOutputReverse,
                BarreConstants.kTimeout);
        m_moteurPremierJoint.configNominalOutputForward(BarreConstants.kNominalOutputForward,
                BarreConstants.kTimeout);
        m_moteurPremierJoint.configNominalOutputReverse(BarreConstants.kNominalOutputReverse,
                BarreConstants.kTimeout);

        m_moteurDeuxiemeJoint.configPeakOutputForward(BarreConstants.kPeakOutputForward,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configPeakOutputReverse(BarreConstants.kPeakOutputReverse,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configNominalOutputForward(BarreConstants.kNominalOutputForward,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configNominalOutputReverse(BarreConstants.kNominalOutputReverse,
                BarreConstants.kTimeout);

        m_moteurPremierJoint.selectProfileSlot(0, 0); // motion
        m_moteurPremierJoint.config_kP(0, BarreConstants.PremierJoint.kPMotion, BarreConstants.kTimeout);
        m_moteurPremierJoint.config_kI(0, BarreConstants.PremierJoint.kIMotion, BarreConstants.kTimeout);
        m_moteurPremierJoint.config_kD(0, BarreConstants.PremierJoint.kDMotion, BarreConstants.kTimeout);
        m_moteurPremierJoint.config_kF(0, BarreConstants.PremierJoint.kFMotion, BarreConstants.kTimeout);

        m_moteurDeuxiemeJoint.selectProfileSlot(0, 0); // motion
        m_moteurDeuxiemeJoint.config_kP(0, BarreConstants.DeuxiemeJoint.kPMotion, BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.config_kI(0, BarreConstants.DeuxiemeJoint.kIMotion, BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.config_kD(0, BarreConstants.DeuxiemeJoint.kDMotion, BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.config_kF(0, BarreConstants.DeuxiemeJoint.kFMotion, BarreConstants.kTimeout);

        m_moteurPremierJoint.configMotionCruiseVelocity(
                BarreConstants.PremierJoint.kVitesse / FacteursConversion.Barre.PremierJoint.fVelocity,
                BarreConstants.kTimeout);
        m_moteurPremierJoint.configMotionAcceleration(
                BarreConstants.PremierJoint.kAcceleration / FacteursConversion.Barre.PremierJoint.fAcceleration,
                BarreConstants.kTimeout);

        m_moteurDeuxiemeJoint.configMotionCruiseVelocity(
                BarreConstants.DeuxiemeJoint.kVitesse / FacteursConversion.Barre.DeuxiemeJoint.fVelocity,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configMotionAcceleration(
                BarreConstants.DeuxiemeJoint.kAcceleration / FacteursConversion.Barre.DeuxiemeJoint.fAcceleration,
                BarreConstants.kTimeout);

        m_moteurPremierJoint.configVoltageCompSaturation(BarreConstants.kVoltageForCompensation,
                BarreConstants.kTimeout);
        m_moteurPremierJoint.enableVoltageCompensation(true);

        m_moteurDeuxiemeJoint.configVoltageCompSaturation(BarreConstants.kVoltageForCompensation,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.enableVoltageCompensation(true);

        m_moteurPremierJoint.configPeakCurrentLimit(BarreConstants.kPeakCurrentLimit, BarreConstants.kTimeout);
        m_moteurPremierJoint.configPeakCurrentDuration(BarreConstants.kPeakCurrentDuration, BarreConstants.kTimeout);
        m_moteurPremierJoint.configContinuousCurrentLimit(BarreConstants.kContinuousCurrent, BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.enableCurrentLimit(true);

        m_moteurDeuxiemeJoint.configPeakCurrentLimit(BarreConstants.kPeakCurrentLimit, BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configPeakCurrentDuration(BarreConstants.kPeakCurrentDuration, BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configContinuousCurrentLimit(BarreConstants.kContinuousCurrent,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.enableCurrentLimit(true);

        m_moteurPremierJoint.configForwardSoftLimitThreshold(
                BarreConstants.PremierJoint.kForwardSoftLimit / FacteursConversion.Barre.PremierJoint.fPosition,
                BarreConstants.kTimeout);
        m_moteurPremierJoint.configReverseSoftLimitThreshold(
                BarreConstants.PremierJoint.kReverseSoftLimit / FacteursConversion.Barre.PremierJoint.fPosition,
                BarreConstants.kTimeout);
        m_moteurPremierJoint.configForwardSoftLimitEnable(true, BarreConstants.kTimeout);
        m_moteurPremierJoint.configReverseSoftLimitEnable(true, BarreConstants.kTimeout);

        m_moteurDeuxiemeJoint.configForwardSoftLimitThreshold(
                BarreConstants.DeuxiemeJoint.kForwardSoftLimit / FacteursConversion.Barre.DeuxiemeJoint.fPosition,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configReverseSoftLimitThreshold(
                BarreConstants.DeuxiemeJoint.kReverseSoftLimit / FacteursConversion.Barre.DeuxiemeJoint.fPosition,
                BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configForwardSoftLimitEnable(true, BarreConstants.kTimeout);
        m_moteurDeuxiemeJoint.configReverseSoftLimitEnable(true, BarreConstants.kTimeout);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("test", 123);
        SmartDashboard.putNumber("Premier Joint Position", getJointPosition(m_moteurPremierJoint));
        SmartDashboard.putNumber("Deuxieme Joint Position", getJointPosition(m_moteurDeuxiemeJoint));
        SmartDashboard.putNumber("Premier Joint Velocity", getJointVelocity(m_moteurPremierJoint));
        SmartDashboard.putNumber("Deuxieme Joint Velocity", getJointVelocity(m_moteurDeuxiemeJoint));

    }

    public void manualOutputPremierJoint(double percent) {
        manualOutputJoint(m_moteurPremierJoint, percent);
    }

    public void manualOutputDeuxiemeJoint(double percent) {
        manualOutputJoint(m_moteurDeuxiemeJoint, percent);
    }

    /**
     * @param angle
     *            desired angle (1/10 degre)
     */
    public void setPremierJointAngle(double angle) {
        setJointAngle(m_moteurPremierJoint, angle);
    }

    /**
     * @param angle
     *            desired angle (1/10 degre)
     */
    public void setDeuxiemeJointAngle(double angle) {
        setJointAngle(m_moteurDeuxiemeJoint, angle);
    }

    /**
     * @param targetAngle
     *            target angle (1/10 degre)
     * @return is premier joint position within threshold of target
     */
    public boolean isPremierJointAtTargetAngle(double targetAngle) {
        return isJointAtTargetAngle(m_moteurPremierJoint, targetAngle);
    }

    /**
     * @param targetAngle
     *            target angle (1/10 degre)
     * @return is deuxieme joint position within threshold of target
     */
    public boolean isDeuxiemeJointAtTargetAngle(double targetAngle) {
        return isJointAtTargetAngle(m_moteurDeuxiemeJoint, targetAngle);
    }

    public void seedEncoderPremierJoint() {
        seedEncoder(m_moteurPremierJoint);
    }

    public void seedEncoderDeuxiemeJoint() {
        seedEncoder(m_moteurDeuxiemeJoint);
    }

    public void keepCurrentAnglePremierJoint() {
        setJointAngle(m_moteurPremierJoint, getJointPosition(m_moteurPremierJoint));
    }

    public void keepCurrentAngleDeuxiemeJoint() {
        setJointAngle(m_moteurDeuxiemeJoint, getJointPosition(m_moteurDeuxiemeJoint));
    }

    private void seedEncoder(TalonSRX moteurJoint) {
        int absoluteEncoderPosition = moteurJoint.getSensorCollection().getPulseWidthPosition();
        if (moteurJoint.equals(m_moteurDeuxiemeJoint)) {
            absoluteEncoderPosition *= -1;
        }

        absoluteEncoderPosition = absoluteEncoderPosition % 4096;
        if (absoluteEncoderPosition < 0) {
            absoluteEncoderPosition += 4096;
        }

        double offset = (moteurJoint.equals(m_moteurPremierJoint))
                ? (BarreConstants.PremierJoint.kAbsoluteEncoderOffset / FacteursConversion.Barre.PremierJoint.fPosition)
                : (BarreConstants.DeuxiemeJoint.kAbsoluteEncoderOffset
                        / FacteursConversion.Barre.DeuxiemeJoint.fPosition);
        double relativeEncoderPosition = absoluteEncoderPosition + offset;

        if (relativeEncoderPosition < 0) {
            relativeEncoderPosition += 4096;
        }
        relativeEncoderPosition = (int) relativeEncoderPosition % 4096;
        moteurJoint.setSelectedSensorPosition(relativeEncoderPosition);
    }

    private boolean isJointAtTargetAngle(TalonSRX moteurJoint, double targetAngle) {
        if ((Math.abs((getJointPosition(moteurJoint)) -
                targetAngle) <= BarreConstants.kAngleThreshold)) {
            return true;
        }
        return false;
    }

    /** return joint position in 1/10 degre using conversion factors */
    private double getJointPosition(TalonSRX moteurJoint) {
        return moteurJoint.getSelectedSensorPosition()
                * (moteurJoint.equals(m_moteurPremierJoint) ? FacteursConversion.Barre.PremierJoint.fPosition
                        : FacteursConversion.Barre.DeuxiemeJoint.fPosition);
    }

    /** 1/10 degre */
    private double getJointVelocity(TalonSRX moteurJoint) {
        return moteurJoint.getSelectedSensorVelocity()
                * (moteurJoint.equals(m_moteurPremierJoint) ? FacteursConversion.Barre.PremierJoint.fVelocity
                        : FacteursConversion.Barre.DeuxiemeJoint.fVelocity);
    }

    private void manualOutputJoint(TalonSRX moteurJoint, double percent) {
        double kAF = moteurJoint.equals(m_moteurPremierJoint)
                ? computeKAFPremierJoint(getJointPosition(m_moteurPremierJoint))
                : 0;
        moteurJoint.set(ControlMode.PercentOutput, percent, DemandType.ArbitraryFeedForward, kAF);
    }

    private void setJointAngle(TalonSRX moteurJoint, double angle) {
        angle = angle / (moteurJoint.equals(m_moteurPremierJoint) ? FacteursConversion.Barre.PremierJoint.fPosition
                : FacteursConversion.Barre.DeuxiemeJoint.fPosition); // convert to ticks

        double kAF = moteurJoint.equals(m_moteurPremierJoint)
                ? computeKAFPremierJoint(getJointPosition(m_moteurPremierJoint))
                : 0;

        moteurJoint.set(ControlMode.MotionMagic, angle, DemandType.ArbitraryFeedForward, kAF);
    }

    /**
     * @param angle
     *            current joint position (1/10 degre)
     */
    private double computeKAFPremierJoint(double angle) {
        double val = BarreConstants.PremierJoint.kMaxAF * Math.cos(Math.toRadians(angle / 10));
        SmartDashboard.putNumber("Current kAF Value Premier Joint", val);
        return val;
    }
}
