package frc.OscarLib.lib.Swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;

public final class SwerveConfigs {
        public final static TalonFXConfiguration angle;
        public final static TalonFXConfiguration drive;
        public final static CANcoderConfiguration canCoder;
        public final static Pigeon2Configuration pigeon;
        public final static SwerveModuleConstants Mod0;
        public final static SwerveModuleConstants Mod1;
        public final static SwerveModuleConstants Mod2;
        public final static SwerveModuleConstants Mod3;

        static {
                angle = new TalonFXConfiguration();
                drive = new TalonFXConfiguration();
                canCoder = new CANcoderConfiguration();
                pigeon = new Pigeon2Configuration();

                Mod0 = new SwerveModuleConstants();
                Mod1 = new SwerveModuleConstants();
                Mod2 = new SwerveModuleConstants();
                Mod3 = new SwerveModuleConstants();

                /** Swerve CANCoder Configuration */
                canCoder.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
                canCoder.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

                /** Swerve Angle Motor Configuration */
                angle.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                angle.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                angle.Feedback.SensorToMechanismRatio = SwerveConstants.angleGearRatio; // Updated gear ratio
                angle.ClosedLoopGeneral.ContinuousWrap = true;
                angle.CurrentLimits.SupplyCurrentLimitEnable = true;
                angle.CurrentLimits.SupplyCurrentLimit = 45;
                angle.CurrentLimits.SupplyCurrentLowerLimit = 80;
                angle.CurrentLimits.SupplyCurrentLowerTime = 0.1;
                angle.CurrentLimits.StatorCurrentLimitEnable = true;
                angle.CurrentLimits.StatorCurrentLimit = 80;
                angle.Slot0.kP = 180.0; // Updated PID constant
                angle.Slot0.kI = 0.0; // Updated PID constant
                angle.Slot0.kD = 0.0; // Updated PID constant

                angle.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0;
                angle.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0;

                angle.Voltage.PeakForwardVoltage = 16.0;
                angle.Voltage.PeakReverseVoltage = -16.0;

                /** Swerve Drive Motor Configuration */
                drive.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
                drive.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                drive.Feedback.SensorToMechanismRatio = SwerveConstants.driveGearRatio; // Updated gear ratio
                drive.CurrentLimits.SupplyCurrentLimitEnable = true;
                drive.CurrentLimits.SupplyCurrentLimit = 85;
                drive.CurrentLimits.SupplyCurrentLowerLimit = 60;
                drive.CurrentLimits.SupplyCurrentLowerTime = 0.2;
                drive.CurrentLimits.StatorCurrentLimitEnable = true;
                drive.CurrentLimits.StatorCurrentLimit = 80;

                drive.Voltage.PeakForwardVoltage = 16.0;
                drive.Voltage.PeakReverseVoltage = -16.0;

                drive.Slot0.kP = 2.0; // Updated PID constant
                drive.Slot0.kI = 0.0; // Updated PID constant
                drive.Slot0.kD = 0.0; // Updated PID constant
                drive.Slot1.kP = 2.0;
                drive.Slot1.kI = 2.0;
                drive.Slot1.kD = 2.0;
                drive.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.2; // Updated ramp period
                drive.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.2; // Updated ramp period
                drive.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.02; // Updated ramp period
                drive.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02; // Updated ramp period

                Mod0.EncoderId = 12;
                Mod0.DriveMotorId = 4;
                Mod0.SteerMotorId = 3;
                Mod0.EncoderOffset = Rotation2d.fromRotations(0.452881).getRotations(); // Updated offset

                // Module 1 Configuration (Front Right)
                Mod1.EncoderId = 13;
                Mod1.DriveMotorId = 6;
                Mod1.SteerMotorId = 5;
                Mod1.EncoderOffset = Rotation2d.fromRotations(0.043213 + 0.5).getRotations(); // Updated offset

                // Module 2 Configuration (Back Left)
                Mod2.EncoderId = 11;
                Mod2.DriveMotorId = 2;
                Mod2.SteerMotorId = 1;
                Mod2.EncoderOffset = Rotation2d.fromRotations(0.320068).getRotations(); // Updated offset

                // Module 3 Configuration (Back Right)
                Mod3.EncoderId = 14;
                Mod3.DriveMotorId = 8;
                Mod3.SteerMotorId = 7;
                Mod3.EncoderOffset = Rotation2d.fromRotations(0.255127).getRotations(); // Updated offset

                /** Pigeon Configuration */
                pigeon.MountPose.MountPoseYaw = 0.0; // Updated yaw angle
        }
}