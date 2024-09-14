package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_topMotor = new CANSparkMax(IntakeConstants.kTopMotorID, MotorType.kBrushless);
    private final CANSparkMax m_bottomMotor = new CANSparkMax(IntakeConstants.kBottomMotorID, MotorType.kBrushless);

    private final DigitalInput m_capteurInterieurIntake = new DigitalInput(IntakeConstants.kCapteurID);

    public Intake() {
        m_topMotor.setInverted(true);
        m_bottomMotor.setInverted(true);

        m_topMotor.setCANTimeout(IntakeConstants.kTimeout);
        m_bottomMotor.setCANTimeout(IntakeConstants.kTimeout);

        // See https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces for docs
        m_topMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 157);
        m_bottomMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 163);

        m_topMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 157);
        m_bottomMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 163);

        m_topMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 157);
        m_bottomMotor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 163);

        m_topMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_bottomMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        m_topMotor.enableVoltageCompensation(IntakeConstants.kVoltageForCompensation);
        m_bottomMotor.enableVoltageCompensation(IntakeConstants.kVoltageForCompensation);

        m_topMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        m_bottomMotor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);

        // m_topMotor.burnFlash();
        // m_bottomMotor.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is Object In Intake", isObjectInIntake());
    }

    public boolean isObjectInIntake() {
        return !(m_capteurInterieurIntake.get()); // si vrai, pas d'objet
    }

    public void setIntake(boolean on, boolean reversed, boolean forShot) {
        double voltage = 0;
        if (forShot || reversed) {
            voltage = IntakeConstants.kVoltageIntakeShot;
        } else {
            voltage = IntakeConstants.kVoltageIntakeCommand;
        }
        if (reversed) {
            voltage *= -1;
        }
        if (!on) {
            voltage = 0;
        }
        m_topMotor.setVoltage(voltage);
        m_bottomMotor.setVoltage(voltage);
    }
}
