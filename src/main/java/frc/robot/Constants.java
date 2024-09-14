// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

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
                /** ticks * factor -> 1/10 degres */
                public static final double fPosition = 1800.0 / 4096.0;
                /** ticks/100ms * factor -> 1/10 degres/s */
                public static final double fVelocity = 18000.0 / 4096.0;
                /** ticks/100ms/s * factor -> 1/10 degres/s/s */
                public static final double fAcceleration = 18000.0 / 4096.0;
            }

            public static class DeuxiemeJoint {
                /** ticks * factor -> 1/10 degres */
                public static final double fPosition = 3600.0 / 4096.0;
                /** ticks/100ms * factor -> 1/10 degres/s */
                public static final double fVelocity = 36000.0 / 4096.0;
                /** ticks/100ms/s * factor -> 1/10 degres/s/s */
                public static final double fAcceleration = 36000.0 / 4096.0;
            }
        }

        public static class Shooter {
            public static class ShooterWheels {
                /** ticks * factor -> wheel rotations */
                public static final double fPosition = 1.0 / 42.0;
            }

            public static class ShooterAngle {
                /** ticks * factor -> 1/10 degre */
                public static final double fPosition = 3600.0 / 4096.0;
                /** ticks/100ms * factor -> 1/10 degre/s */
                public static final double fVelocity = 36000.0 / 4096.0;
                /** ticks/100ms/s * factor -> 1/10 degre/s/s */
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

    public static class DriveConstants {
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

    public static class VisionConstants {
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

    public static class PoseEstimationConstants {
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

    public static class BarreConstants {
        /** ms */
        public static final int kTimeout = 50;

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

    public static class IntakeConstants {
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
        public static final int kTimeout = 50;
    }

    public static class LedConstants {
        public static final int kLedChannel = 9;
        public static final int kNumLeds = 12;

        /** moves by 1 LED each n 20-millis periods */
        public static final int kSweepPrescale = 6;

        /** switches LEDs each n 20-millis periods */
        public static final int kAlternatePrescale = 15;

        /** switches LEDs each n 20-millis periods */
        public static final int kFlashPrescale = 5;

        public static final int kNumSweepFullOnLeds = 8;

        /** number of LEDs for 'in position' starting indicator */
        public static final int kNumIndicatorLeds = 2;

        /** R, G, B */
        public static record Color(int red, int green, int blue) {
        }

        public static class Colors {
            public static final Color off = new Color(0, 0, 0);
            public static final Color redAlliance = new Color(255, 0, 0);
            public static final Color blueAlliance = new Color(0, 0, 255);
            public static final Color noteInIntake = new Color(255, 15, 0);
            public static final Color noteSeen = new Color(255, 10, 150);
            public static final Color robotInRange = new Color(0, 255, 0);
            public static final Color robotInStartingPositionXY = new Color(0, 255, 0);
            public static final Color robotInStartingPositionAngle = new Color(255, 105, 180);
        }

        public static enum Animation {
            ALLIANCE, SPLIT,
        }
    }

    public static class ShooterConstants {

        public static enum ShooterState {
            INIT, WAITING_FOR_SUBSYTEMS, MOVE_NOTE_IN_SHOOTER, WAITING_FOR_NOTE_TO_ENTER, WAITING_FOR_NOTE_TO_EXIT, WAITING_FOR_END, COMPLETE, NO_NOTE,
        }

        public static final int kCapteurID = 1;

        /** seconds */
        public static final double kTimerThresholdAfterExit = 0.2;

        /**
         * a record used for containing measured shooter data to use to interpolate
         * speed and angle
         * 
         * @param distance
         *            distance to the speaker (meters)
         * @param speed
         *            wheel speeds (RPM)
         * @param angle
         *            shooter angle (1/10 degree)
         */
        public static record ShooterData(double distance, double speed, double angle) {
        }

        public static final List<ShooterData> kShooterSpeedAndAngleAccordingToDistance = List
                .of(new ShooterData(1.35, 5500, 650), new ShooterData(1.60, 5500, 600),
                        new ShooterData(1.90, 5500, 545), new ShooterData(2.20, 5500, 485),
                        new ShooterData(2.52, 5500, 445), new ShooterData(2.81, 5500, 415),
                        new ShooterData(3.11, 5500, 400), new ShooterData(3.40, 5500, 387.5),
                        new ShooterData(3.90, 5500, 370), new ShooterData(7.00, 5500, 365),
                        new ShooterData(7.01, 4500, 450));

        public static class ShooterWheels {
            public static final int kLeftMotorID = 16;
            public static final int kRightMotorID = 17;

            /** RPM */
            public static final double kSpeedManualSpeaker = 5500;
            /** RPM */
            public static final double kSpeedAmp = 500;
            /** RPM */
            public static final double kSpeedTrap = 1575;
            /** RPM */
            public static final double kSpeedThreshold = 100;

            public static final double kPLeftFlywheel = 0.00025;
            public static final double kILeftFlywheel = 0.000001;
            public static final double kDLeftFlywheel = 0.01;
            public static final double kFFLeftFlywheel = 0;

            public static final double kPRightFlywheel = 0.00025;
            public static final double kIRightFlywheel = 0.000001;
            public static final double kDRightFlywheel = 0.01;
            public static final double kFFRightFlywheel = 0;

            /** amps */
            public static final int kCurrentLimit = 50;
            /** volts */
            public static final int kVoltageForCompensation = 12;

            /** RPM */
            public static final double kSpeedDifferenceForSpin = 500;

            /** seconds */
            public static final double kMaxTimeForRevUp = 1.5;
        }

        public static class ShooterAngle {
            public static final int kMotorID = 18;
            /** 1/10 degre */
            public static final double kAbsoluteEncoderOffset = -2078.12;

            /** 1/10 degre */
            public static final double kManualSpeakerAngle = 650;

            /** volts */
            public static final int kVoltageForCompensation = 10;
            /** ms */
            public static final int kTimeout = 50;

            public static final double kNominalOutputForward = 0;
            public static final double kPeakOutputForward = 1;
            public static final double kNominalOutputReverse = 0;
            public static final double kPeakOutputReverse = -1;

            public static final double kPPositionAngle = 7.25;
            public static final double kIPositionAngle = 0.015;
            public static final double kDPositionAngle = 25.0;
            public static final double kFPositionAngle = 0;
            public static final double kMaxAF = 0;

            /** 1/10 degre/s */
            public static final double kVitesse = 1000;
            /** 1/10 degre/s/s */
            public static final double kAcceleration = 4000;

            /** 1/10 degre */
            public static final double kAngleThreshold = 7.5;

            /** amperes */
            public static final int kPeakCurrentLimit = 9;
            /** ms */
            public static final int kPeakCurrentDuration = 0;
            /** amperes */
            public static final int kContinuousCurrent = 9;

            /** 1/10 degre */
            public static final double kForwardSoftLimit = 785;
            /** 1/10 degre */
            public static final double kReverseSoftLimit = 200;

            /** 1/10 degre */
            public static final double kAngleAmp = 640;
            /** 1/10 degre */
            public static final double kAngleTrap = 770;
            /** 1/10 degre */
            public static final double kIntermediateAngle = 450;

        }
    }
}
