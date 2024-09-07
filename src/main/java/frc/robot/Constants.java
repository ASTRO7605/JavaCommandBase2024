// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Conversion {
        public static final double fDegreesToRad = Math.PI / 180; // degrees * factor -> rad
    }

    public static class OperatorConstants {
        public static final int kTurnStickID = 0;
        public static final int kThrottleStickID = 1;
        public static final int kCopilotControllerID = 2;
        public static final double kAxisThreshold = 0.5;
    }

    public static class DriveConstant {
        public static final double kDrivingGearRatio = 6.75;
        public static final double kTurningGearRatio = 150.0 / 7.0;

        /** meters */
        public static final double kWheelDiameter = 0.09689;
        /** meters */
        public static final double kWheelCirconf = kWheelDiameter * Math.PI;
        /** meters */
        public static final double kDistanceRightAndLeftWheels = 0.57785;
        /** meters */
        public static final double kDistanceFrontAndRearWheels = 0.66675;
        /** meters */
        public static final double kChassisRadius = 0.44;

        /** important to follow order: front right, front left, rear left, rear right */
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kDistanceFrontAndRearWheels / 2, -kDistanceRightAndLeftWheels / 2),
                new Translation2d(kDistanceFrontAndRearWheels / 2, kDistanceRightAndLeftWheels / 2),
                new Translation2d(-kDistanceFrontAndRearWheels / 2, kDistanceRightAndLeftWheels / 2),
                new Translation2d(-kDistanceFrontAndRearWheels / 2, -kDistanceRightAndLeftWheels / 2));

        public static final boolean kGyroReversed = false;
        /** amps */
        public static final double kCurrentLimit = 50;
        /** m/s */
        public static final double kMaxTeleopSpeed = 4.25;
        /** radians/s */
        public static final double kMaxTeleopAngularSpeed = Math.PI * 1.5;

        /** m/s */
        public static final double kMaxAutoSpeed = 4.25;
        public static final double kPAutoMovementController = 5.5;
        public static final double kPAutoThetaController = 3.25;

        public static final double kDirectionSlewRate = 1.2; // radians per second *valeurs à tester et changer
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        public static final double kControllerMovementDeadband = 0.1; // valeur minimum qu'on recoit des controllers
        public static final double kControllerRotationDeadband = 0.1;

        public static final double kPDriving = 0.1;
        public static final double kIDriving = 0.0005;
        public static final double kDDriving = 0;
        public static final double kFFDriving = 0.15;
        public static final double kDrivingMinInput = -1.0;
        public static final double kDrivingMaxInput = 1.0;

        public static final double kPTurning = 1.65;
        public static final double kITurning = 0;
        public static final double kDTurning = 0;
        public static final double kFFTurning = 0;
        public static final double kTurningMinInput = -1.0;
        public static final double kTurningMaxInput = 1.0;

        public static final double kVoltageForCompensation = 10;

        public static final int kPowerDistributionHubID = 1;
        public static final int kFrontRightTurningID = 2;
        public static final int kFrontRightDrivingID = 3;
        public static final int kFrontRightCANcoderID = 4;
        public static final int kFrontLeftTurningID = 5;
        public static final int kFrontLeftDrivingID = 6;
        public static final int kFrontLeftCANcoderID = 7;
        public static final int kRearLeftTurningID = 8;
        public static final int kRearLeftDrivingID = 9;
        public static final int kRearLeftCANcoderID = 10;
        public static final int kRearRightTurningID = 11;
        public static final int kRearRightDrivingID = 12;
        public static final int kRearRightCANcoderID = 13;
    }
}
