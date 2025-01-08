package frc.robot.subsystemconfigs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.OscarLib.lib.Swerve.SwerveConstants;
import frc.robot.Constants;

public final class ElevatorConfigs {
    public final static TalonFXConfiguration motor1;
    public final static TalonFXConfiguration motor2;

    static {
        motor1 = new TalonFXConfiguration();
        motor2 = new TalonFXConfiguration();



        motor1.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor1.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motor1.Feedback.SensorToMechanismRatio = 0; // Use conversions in code
        motor1.ClosedLoopGeneral.ContinuousWrap = false;
        motor1.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor1.CurrentLimits.SupplyCurrentLimit = 45;
        motor1.CurrentLimits.SupplyCurrentLowerLimit = 80;
        motor1.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        motor1.CurrentLimits.StatorCurrentLimitEnable = true;
        motor1.CurrentLimits.StatorCurrentLimit = 80;
        motor1.Slot0.kP = 1.0; // Updated PID constant
        motor1.Slot0.kI = 0.0; // Updated PID constant
        motor1.Slot0.kD = 0.0; // Updated PID constant
        motor1.Slot1.kP = 999.0;
        motor1.Slot1.kI = 999.0;
        motor1.Slot1.kD = 999.0;
        motor1.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motor1.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motor1.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 36; //set max and min value for system
        motor1.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        motor1.Slot0.kG = 0.0;
        motor1.Slot0.kS = 0.0;
        motor1.Slot0.kV = 0.0;
        motor1.Slot0.kA = 0.0;
        motor1.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        motor1.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motor1.TorqueCurrent.PeakReverseTorqueCurrent = 80;
        motor1.MotionMagic.MotionMagicAcceleration = 40;
        motor1.MotionMagic.MotionMagicCruiseVelocity = 40;
        motor1.MotionMagic.MotionMagicJerk = 20;


        motor2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motor2.Feedback.SensorToMechanismRatio = 0; // Use conversions in code
        motor2.ClosedLoopGeneral.ContinuousWrap = false;
        motor2.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor2.CurrentLimits.SupplyCurrentLimit = 45;
        motor2.CurrentLimits.SupplyCurrentLowerLimit = 80;
        motor2.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        motor2.CurrentLimits.StatorCurrentLimitEnable = true;
        motor2.CurrentLimits.StatorCurrentLimit = 80;
        motor2.Slot0.kP = 1.0; // Updated PID constant
        motor2.Slot0.kI = 0.0; // Updated PID constant
        motor2.Slot0.kD = 0.0; // Updated PID constant
        motor2.Slot1.kP = 999.0;
        motor2.Slot1.kI = 999.0;
        motor2.Slot1.kD = 999.0;
        motor2.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motor2.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motor2.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 36; //set max and min value for system
        motor2.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        motor2.Slot0.kG = 0.0;
        motor2.Slot0.kS = 0.0;
        motor2.Slot0.kV = 0.0;
        motor2.Slot0.kA = 0.0;
        motor2.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        motor2.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motor2.TorqueCurrent.PeakReverseTorqueCurrent = 80;
        motor2.MotionMagic.MotionMagicAcceleration = 40;
        motor2.MotionMagic.MotionMagicCruiseVelocity = 40;
        motor2.MotionMagic.MotionMagicJerk = 20;

    }
}
