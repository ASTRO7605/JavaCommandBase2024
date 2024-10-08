// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.FacteursConversion.Barre;
import frc.robot.subsystems.Base;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Led;
import frc.robot.utils.AutoChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final Base m_base = new Base();
    private final Barre m_barre = new Barre();
    private final Intake m_intake = new Intake();
    // private final Led m_led = new Led();

    private final CommandJoystick m_throttleStick = new CommandJoystick(OperatorConstants.kThrottleStickID);
    private final CommandJoystick m_turnStick = new CommandJoystick(OperatorConstants.kTurnStickID);
    private final CommandXboxController m_copilotController = new CommandXboxController(
            OperatorConstants.kCopilotControllerID);

    private final AutoChooser m_autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        configureNamedCommands();

        m_base.setDefaultCommand(new RunCommand(() -> {
            double dir_x = m_throttleStick.getX();
            double dir_y = m_throttleStick.getY();

            // Convert cartesian vector to polar for circular deadband
            double dir_r = Math.sqrt(Math.pow(dir_x, 2) + Math.pow(dir_y, 2)); // norm of vector
            double dir_theta = Math.atan2(dir_y, dir_x); // direction of vector (rad)

            // Cap norm and add deadband
            if (dir_r < DriveConstants.kControllerMovementDeadband) {
                dir_r = 0.0;
            } else if (dir_r > 1.0) {
                dir_r = 1.0;
            } else {
                dir_r = (dir_r - DriveConstants.kControllerMovementDeadband) /
                        (1 - DriveConstants.kControllerMovementDeadband);
            }

            double turn = 0;
            turn = MathUtil.applyDeadband(m_turnStick.getX(), DriveConstants.kControllerRotationDeadband);

            m_base.drive(-dir_r * Math.sin(dir_theta), -dir_r * Math.cos(dir_theta), -turn, true);
        }, m_base));

        m_autoChooser = new AutoChooser();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(m_exampleSubsystem::exampleCondition)
        //         .onTrue(new ExampleCommand(m_exampleSubsystem));

        m_turnStick.button(5).whileTrue(new RunCommand(() -> {
            var latestCameraPose = m_base.getAveragePoseFromCameras();
            if (latestCameraPose.isPresent()) {
                m_base.resetOdometry(latestCameraPose.get());
            }
        }));
        m_turnStick.button(6).onTrue(new InstantCommand(() -> {
            m_base.resetGyroOffset(true);
        }));

        m_throttleStick.button(5).onTrue(new InstantCommand(() -> m_base.resetGyroOffset(false)));
        m_throttleStick.button(6).onTrue(new InstantCommand(() -> m_base.switchRobotDrivingMode()));
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("test", new PrintCommand("named command test"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelectedAutoCommand();
    }

    public void setIdleModeSwerve(IdleMode idleMode) {
        m_base.setIdleMode(idleMode);
    }

    public void resetGyroOffsetFromAuto() {
        var startingAutoPose = m_autoChooser.getStartingAutoPose();
        if (startingAutoPose.isPresent()) {
            m_base.resetGyroOffset(startingAutoPose.get().getRotation().getRadians());
        } else {
            System.out.print("Starting auto pose null");
        }
    }

    public String getSelectedAutoModeName() {
        return m_autoChooser.getSelectedAutoModeName();
    }

}
