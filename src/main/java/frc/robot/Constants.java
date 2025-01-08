package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.OscarLib.lib.Swerve.SwerveConfigs;
import frc.OscarLib.lib.Swerve.SwerveConstants;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 18;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                             // robot
        public static final double wheelBase = Units.inchesToMeters(21.73); // TODO: This must be tuned to specific
                                                                            // robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.02;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 2.0; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = (0.967 / 12); // TODO: This must be tuned to specific robot
        public static final double driveKV = (5.44 / 12);
        public static final double driveKA = (0.57 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.034 + 0.5);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.456);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.178 + 0.5);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.258);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class OI {
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
    }

    public static final class Elevator {
        public static final int motor1ID = 0;
        public static final int motor2ID = 1;
        public static final int bottomHallEffectSensorID = 0;

        public static final double rotationsToMeters = 0.1; // tweak with actual value from robot

        public static final boolean autoResetElevator = true;
    }

    public static final class Field {
        // Field dimensions (in feet)
        private static final double FIELD_WIDTH = 27.0;
        private static final double FIELD_LENGTH = 54.0;
        private static final double REEF_CENTER_X = FIELD_LENGTH / 2.0;
        private static final double REEF_RED_Y = FIELD_WIDTH / 4.0;
        private static final double REEF_BLUE_Y = 3 * FIELD_WIDTH / 4.0;

        private static final double BARGE_X = FIELD_LENGTH / 2.0;
        private static final double BARGE_RED_Y = REEF_RED_Y + 10.0;
        private static final double BARGE_BLUE_Y = REEF_BLUE_Y - 10.0;
        private static final double CAGE_OFFSET_X = 3.5; // Distance between cages
        private static final double SHALLOW_CAGE_Y_OFFSET = 3.0;
        private static final double DEEP_CAGE_Y_OFFSET = 5.0;

        private static final double CORAL_STATION_Y_OFFSET = 5.0;
        private static final double PROCESSOR_Y_OFFSET = 3.0;

        // Utility method for calculating rotation to reef center
        private static double calculateRotationToCenter(double x, double y, double centerX, double centerY) {
            return Math.toDegrees(Math.atan2(centerY - y, centerX - x));
        }

        // Cage Positions
        public static final Pose2d RED_CAGE_1 = new Pose2d(BARGE_X - CAGE_OFFSET_X, BARGE_RED_Y - SHALLOW_CAGE_Y_OFFSET,
                Rotation2d.fromDegrees(0));
        public static final Pose2d RED_CAGE_2 = new Pose2d(BARGE_X, BARGE_RED_Y - SHALLOW_CAGE_Y_OFFSET,
                Rotation2d.fromDegrees(0));
        public static final Pose2d RED_CAGE_3 = new Pose2d(BARGE_X + CAGE_OFFSET_X, BARGE_RED_Y - SHALLOW_CAGE_Y_OFFSET,
                Rotation2d.fromDegrees(0));

        public static final Pose2d BLUE_CAGE_1 = new Pose2d(BARGE_X - CAGE_OFFSET_X,
                BARGE_BLUE_Y + SHALLOW_CAGE_Y_OFFSET, Rotation2d.fromDegrees(180));
        public static final Pose2d BLUE_CAGE_2 = new Pose2d(BARGE_X, BARGE_BLUE_Y + SHALLOW_CAGE_Y_OFFSET,
                Rotation2d.fromDegrees(180));
        public static final Pose2d BLUE_CAGE_3 = new Pose2d(BARGE_X + CAGE_OFFSET_X,
                BARGE_BLUE_Y + SHALLOW_CAGE_Y_OFFSET, Rotation2d.fromDegrees(180));

        // Coral Station Positions
        public static final Pose2d RED_CORAL_STATION_1 = new Pose2d(0.0, REEF_RED_Y - CORAL_STATION_Y_OFFSET,
                Rotation2d.fromDegrees(90));
        public static final Pose2d RED_CORAL_STATION_2 = new Pose2d(0.0, REEF_RED_Y + CORAL_STATION_Y_OFFSET,
                Rotation2d.fromDegrees(90));

        public static final Pose2d BLUE_CORAL_STATION_1 = new Pose2d(0.0, REEF_BLUE_Y - CORAL_STATION_Y_OFFSET,
                Rotation2d.fromDegrees(-90));
        public static final Pose2d BLUE_CORAL_STATION_2 = new Pose2d(0.0, REEF_BLUE_Y + CORAL_STATION_Y_OFFSET,
                Rotation2d.fromDegrees(-90));

        // Processor Positions
        public static final Pose2d RED_PROCESSOR = new Pose2d(FIELD_LENGTH, REEF_RED_Y - PROCESSOR_Y_OFFSET,
                Rotation2d.fromDegrees(180));
        public static final Pose2d BLUE_PROCESSOR = new Pose2d(FIELD_LENGTH, REEF_BLUE_Y + PROCESSOR_Y_OFFSET,
                Rotation2d.fromDegrees(180));

        // Net Positions
        public static final Pose2d RED_NET = new Pose2d(BARGE_X, BARGE_RED_Y + DEEP_CAGE_Y_OFFSET,
                Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_NET = new Pose2d(BARGE_X, BARGE_BLUE_Y - DEEP_CAGE_Y_OFFSET,
                Rotation2d.fromDegrees(180));

        private static final double REEF_RADIUS = 5.5; // Approximate reef scoring radius
        private static final double BRANCH_ANGLE = 60.0; // Degrees between adjacent branches
        private static final double TROUGH_Y_OFFSET = 2.0;

        // Red Alliance Reef Positions
        public static final Pose2d RED_TROUGH_LEFT = new Pose2d(REEF_CENTER_X - TROUGH_Y_OFFSET, REEF_RED_Y,
                Rotation2d.fromDegrees(0));
        public static final Pose2d RED_TROUGH_RIGHT = new Pose2d(REEF_CENTER_X + TROUGH_Y_OFFSET, REEF_RED_Y,
                Rotation2d.fromDegrees(0));

        public static final Pose2d[] RED_BRANCHES = new Pose2d[12];
        static {
            for (int i = 0; i < 12; i++) {
                double angle = Math.toRadians(BRANCH_ANGLE * i);
                double x = REEF_CENTER_X + REEF_RADIUS * Math.cos(angle);
                double y = REEF_RED_Y + REEF_RADIUS * Math.sin(angle);
                double rotation = calculateRotationToCenter(x, y, REEF_CENTER_X, REEF_RED_Y);
                RED_BRANCHES[i] = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
            }
        }

        // Blue Alliance Reef Positions
        public static final Pose2d BLUE_TROUGH_LEFT = new Pose2d(REEF_CENTER_X - TROUGH_Y_OFFSET, REEF_BLUE_Y,
                Rotation2d.fromDegrees(180));
        public static final Pose2d BLUE_TROUGH_RIGHT = new Pose2d(REEF_CENTER_X + TROUGH_Y_OFFSET, REEF_BLUE_Y,
                Rotation2d.fromDegrees(180));

        public static final Pose2d[] BLUE_BRANCHES = new Pose2d[12];
        static {
            for (int i = 0; i < 12; i++) {
                double angle = Math.toRadians(BRANCH_ANGLE * i);
                double x = REEF_CENTER_X + REEF_RADIUS * Math.cos(angle);
                double y = REEF_BLUE_Y - REEF_RADIUS * Math.sin(angle);
                double rotation = calculateRotationToCenter(x, y, REEF_CENTER_X, REEF_BLUE_Y);
                BLUE_BRANCHES[i] = new Pose2d(x, y, Rotation2d.fromDegrees(rotation));
            }
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = 1.5 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 1.5 * Math.PI;

        public static final PathConstraints GLOBAL_CONSTRAINTS = new PathConstraints(kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared, kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularSpeedRadiansPerSecondSquared);

        public static PIDConstants translationConstants = new PIDConstants(4.0, 0.0, 0.0);
        public static PIDConstants rotationConstants = new PIDConstants(4.0, 0, 0.0);

        public static RobotConfig config = new RobotConfig(SwerveConstants.massKgs, 6.0, new ModuleConfig(SwerveConstants.wheelDiameter/2, SwerveConstants.maxSpeed, 1.542, DCMotor.getKrakenX60Foc(1).withReduction(SwerveConstants.driveGearRatio), SwerveConfigs.drive.CurrentLimits.SupplyCurrentLimit, 1), SwerveConstants.modPositions);


    }
    public static class AdvancedTractionControlConstants {

        public enum TractionControlLevel {
            /**
             * Fully disable traction control by setting kP=0, slip ratio=0,
             * and letting the min/max torque scale = 1.0 (no scaling).
             */
            OFF(
                    0.0, // slip ratio
                    0.0, // kP
                    0.0, // kI
                    0.0, // kD
                    1.0, // min torque scale
                    1.0 // max torque scale
            ),

            /**
             * Level 4: Very restrictive (large kP, small slip ratio).
             * The min torque scale is quite low, meaning it can cut power aggressively.
             */
            LEVEL4(
                    0.05, // kDesiredSlipRatio
                    1.0, // kP
                    0.0, // kI
                    0.05, // kD
                    0.2, // kMinTorqueScale
                    0.7 // kMaxTorqueScale
            ),

            /**
             * Level 3: Still fairly restrictive but a bit less so.
             */
            LEVEL3(
                    0.07, // kDesiredSlipRatio
                    0.8, // kP
                    0.0, // kI
                    0.05, // kD
                    0.2, // kMinTorqueScale
                    0.8 // kMaxTorqueScale
            ),

            /**
             * Level 2: Moderate traction control.
             */
            LEVEL2(
                    0.10, // kDesiredSlipRatio
                    0.7, // kP
                    0.0, // kI
                    0.05, // kD
                    0.2, // kMinTorqueScale
                    0.9 // kMaxTorqueScale
            ),

            /**
             * Level 1: Minimal traction control, only reduces slip a little.
             */
            LEVEL1(
                    0.15, // kDesiredSlipRatio
                    0.5, // kP
                    0.0, // kI
                    0.05, // kD
                    0.2, // kMinTorqueScale
                    1.0 // kMaxTorqueScale
            );

            // ------------------------------
            // Fields for each traction level
            // ------------------------------
            private final double desiredSlipRatio;
            private final double kP;
            private final double kI;
            private final double kD;
            private final double minTorqueScale;
            private final double maxTorqueScale;

            // Enum constructor
            TractionControlLevel(double desiredSlipRatio, double kP, double kI, double kD,
                    double minTorqueScale, double maxTorqueScale) {
                this.desiredSlipRatio = desiredSlipRatio;
                this.kP = kP;
                this.kI = kI;
                this.kD = kD;
                this.minTorqueScale = minTorqueScale;
                this.maxTorqueScale = maxTorqueScale;
            }

            /**
             * Apply this traction control level by overwriting
             * the static fields in AdvancedTractionControlConstants.
             */
            public void apply() {
                AdvancedTractionControlConstants.kDesiredSlipRatio = this.desiredSlipRatio;
                AdvancedTractionControlConstants.kP = this.kP;
                AdvancedTractionControlConstants.kI = this.kI;
                AdvancedTractionControlConstants.kD = this.kD;
                AdvancedTractionControlConstants.kMinTorqueScale = this.minTorqueScale;
                AdvancedTractionControlConstants.kMaxTorqueScale = this.maxTorqueScale;
            }
        }

        // Desired slip ratio (this is the "target" we want to achieve)
        public static double kDesiredSlipRatio = 0.10;

        // PID gains for slip ratio control
        public static double kP = 0.8;
        public static double kI = 0.0;
        public static double kD = 0.05;

        // Additional constraints
        public static double kMaxTorqueScale = 1.0;
        public static double kMinTorqueScale = 0.2;

        // Speed below which slip ratio control is not used
        public static double kSpeedDeadband = 0.1; // m/s
    }

}
