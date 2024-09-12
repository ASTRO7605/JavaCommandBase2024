// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
    public static class FacteursConversion {
        /** inches * factor -> meters */
        public static final double fInchesToMeters = 0.0254;

        public static class Barre {
            public static class PremierJoint {
                /** ticks * factor -> 0.1 degres */
                public static final double fPosition = 1800.0 / 4096.0;
                /** ticks/100ms * factor -> 0.1 degres/s */
                public static final double fVelocity = 18000.0 / 4096.0;
                /** ticks/100ms/s * factor -> 0.1 degres/s/s */
                public static final double fAcceleration = 18000.0 / 4096.0;
            }

            public static class DeuxiemeJoint {
                /** ticks * factor -> 0.1 degres */
                public static final double fPosition = 3600.0 / 4096.0;
                /** ticks/100ms * factor -> 0.1 degres/s */
                public static final double fVelocity = 36000.0 / 4096.0;
                /** ticks/100ms/s * factor -> 0.1 degres/s/s */
                public static final double fAcceleration = 36000.0 / 4096.0;
            }
        }
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
                new Translation2d(kDistanceFrontAndRearWheels / 2,
                        -kDistanceRightAndLeftWheels / 2),
                new Translation2d(kDistanceFrontAndRearWheels / 2, kDistanceRightAndLeftWheels / 2),
                new Translation2d(-kDistanceFrontAndRearWheels / 2,
                        kDistanceRightAndLeftWheels / 2),
                new Translation2d(-kDistanceFrontAndRearWheels / 2,
                        -kDistanceRightAndLeftWheels / 2));

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

        /** seconds */
        public static final double kTimeBeforeCoast = 1.5;

        public static final double kPThetaRobot = 525;
        public static final double kIThetaRobot = 0.01;
        public static final double kDThetaRobot = 25;

        public static final double kPXYRobot = 0.0125;
        public static final double kIXYRobot = 0;
        public static final double kDXYRobot = 0;

        public static final double kMaxAutoAlignSpeedY = 0.2;
        public static final double kMaxAutoAlignSpeedX = 0.1;

        /** radians */
        public static final double kThresholdRobotAngle = Math.toRadians(1.5);
        public static final double kThresholdTimerAutoAlign = 0.1;

        /** meters */
        public static final double kThresholdSpeakerInRangeToShoot = 3.9;
        /** meters */
        public static final double kThresholdSpeakerInRangeToStartWheels = 6;

        public static final double kTimeForProjectionInFutureDistance = 0.35;
        public static final double kTimeForProjectionInFutureRotation = 0.25;

        /** radians */
        public static final double kBlueSourceApproachAngle = Math.toRadians(120);
        /** radians */
        public static final double kRedSourceApproachAngle = Math.toRadians(60);

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

    public static class VisionConstant {
        public static final String kTableNameRight = "photonvision-a";
        public static final String kTableNameLeft = "photonvision-b";
        public static final String kTableNameLimelight = "limelight";
        public static final double kAmbiguityThreshold = 0.4;
        public static final Transform3d kRightCameraTransform = new Transform3d(
                new Translation3d(-0.292, -0.176, 0.683),
                new Rotation3d(0, Math.toRadians(-32), Math.toRadians(-24.6)));
        public static final Transform3d kLeftCameraTransform = new Transform3d(
                new Translation3d(-0.292, 0.189, 0.683),
                new Rotation3d(0, Math.toRadians(-32), Math.toRadians(24.6)));

        public static enum StageAprilTagIDs {
            RED_SOURCE_SIDE(11), RED_SPEAKER_SIDE(12), RED_MIDDLE_SIDE(13), BLUE_MIDDLE_SIDE(
                    14), BLUE_SPEAKER_SIDE(15), BLUE_SOURCE_SIDE(16);

            private final int id;

            // Constructor to assign values to the enum constants
            StageAprilTagIDs(int id) {
                this.id = id;
            }

            // Getter to retrieve the associated value
            public int getId() {
                return id;
            }
        }
    }

    public static class PoseEstimationConstant {
        /** x(m), y(m), theta(rad) */
        public static final double[] kStateStdDevs = { 0.12, 0.12, 0.008 };
        public static final double[] kVisionStdDevsDefault = { 0.8, 0.8, 0.99 };

        public static final double[] kVisionStdDevsPerMeterBase = { 0.4, 0.4, 0.95 };
        public static final double[] kVisionStdDevsPerAmbiguityPerMeter = { 10.0, 10.0, 500.0 };

        public static final Translation2d kBlueSpeakerPoseMeters = new Translation2d(
                -1.5 * FacteursConversion.fInchesToMeters,
                218.42 * FacteursConversion.fInchesToMeters);
        public static final Translation2d kRedSpeakerPoseMeters = new Translation2d(
                652.73 * FacteursConversion.fInchesToMeters,
                218.42 * FacteursConversion.fInchesToMeters);
    }

    public static class BarreConstant {
        public static final int kTimeoutMs = 50;

        public static final double kNominalOutputForward = 0;
        public static final double kPeakOutputForward = 1;
        public static final double kNominalOutputReverse = 0;
        public static final double kPeakOutputReverse = -1;

        public static final double kVoltageForCompensation = 10;

        /** amperes */
        public static final int kPeakCurrentLimit = 9;
        /** ms */
        public static final int kPeakCurrentDuration = 0;
        /** amperes */
        public static final int kContinuousCurrent = 9;

        /** 1/10 degre */
        public static final double kAngleThreshold = 15;

        /** seconds */
        public static final double kTimerThresholdComeDown = 0.5;

        public static class PremierJoint {
            public static final int kMoteurID = 19;

            /** 1/10 degre */
            public static final double kAbsoluteEncoderOffset = -940.1;

            public static final double kPMotion = 1.75;
            public static final double kIMotion = 0;
            public static final double kDMotion = 20;
            public static final double kFMotion = 0;
            public static final double kMaxAF = 0.065;

            /** 1/10 degre/s */
            public static final double kVitesse = 2750;
            /** 1/10 degre/s/s */
            public static final double kAcceleration = 12000;

            /** 1/10 degre */
            public static final double kForwardSoftLimit = 1200;
            /** 1/10 degre */
            public static final double kReverseSoftLimit = 80;

            public static final double kPourcentageManual = 0.15;

            /** 1/10 degre */
            public static final double kAngleTrapApproach = 350;
            /** 1/10 degre */
            public static final double kAngleTrapIntermediaire = 900;
            /** 1/10 degre */
            public static final double kAngleTrapFinal = 1110;
            /** 1/10 degre */
            public static final double kAngleAmp = 1020;

            /** 1/10 degre */
            public static final double kStartPosition = 80;
        }

        public static class DeuxiemeJoint {
            public static final int kMoteurID = 20;

            /** 1/10 degre */
            public static final double kAbsoluteEncoderOffset = 806;

            public static final double kPMotion = 4;
            public static final double kIMotion = 0;
            public static final double kDMotion = 50;
            public static final double kFMotion = 0;

            /** 1/10 degre/s */
            public static final double kVitesse = 6500;
            /** 1/10 degre/s/s */
            public static final double kAcceleration = 150000;

            /** 1/10 degre */
            public static final double kForwardSoftLimit = 3500;
            /** 1/10 degre */
            public static final double kReverseSoftLimit = 100;

            public static final double kPourcentageManual = 0.16;

            /** 1/10 degre */
            public static final double kAngleTrapApproach = 1900;
            /** 1/10 degre */
            public static final double kAngleTrapFinal = 2660;
            /** 1/10 degre */
            public static final double kAngleAmpApproach = 1600;
            /** 1/10 degre */
            public static final double kAngleAmpFinal = 560;

            /** 1/10 degre */
            public static final double kStartPosition = 900;
        }
    }

    public static class IntakeConstant {
        public static final int kTopMotorID = 14;
        public static final int kBottomMotorID = 15;
        public static final int kCapteurID = 0;
        /** volts */
        public static final double kVoltageForCompensation = 10;
        /** amperes */
        public static final int kCurrentLimit = 50;
        /** volts */
        public static final double kVoltageIntakeShot = 10;
        /** volts */
        public static final double kVoltageIntakeCommand = 6.5;

        /** ms */
        public static final int kTimeoutMs = 50;
    }
}
