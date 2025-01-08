package frc.OscarLib.lib.Swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.OscarLib.lib.Util;
import frc.OscarLib.lib.HardwareDevices.CANCoderFactory;
import frc.OscarLib.lib.HardwareDevices.TalonFactory;



public class SwerveModule implements Sendable {

    public enum SwerveControlMode {
        OPEN_LOOP,
        VELOCITY,
        TORQUE  // i.e., current-based or torque-based control
    }


    public static int s_moduleCount = 0;
    public SwerveModuleConstants constants;

    private final StatusSignal<Angle> _encoderSignal;
    private final StatusSignal<Angle> _anglePositionSignal;
    private final StatusSignal<AngularVelocity> _angleVelocitySignal;
    private final StatusSignal<Angle> _drivePositionSignal;
    private final StatusSignal<AngularVelocity> _driveVelocitySignal;

    public static final int SIGNAL_COUNT = 4;

    public int moduleNumber;
    private double _angleOffset;

    public TalonFactory _angleMotor;
    public TalonFactory _driveMotor;
    public CANCoderFactory _encoder;

    private double _lastAngle;

    private final SimpleMotorFeedforward _driveFeedForward = new SimpleMotorFeedforward(SwerveConstants.drivekS,
            SwerveConstants.drivekV, SwerveConstants.drivekA);

    private final VelocityVoltage _driveVelocity = new VelocityVoltage(0).withEnableFOC(true);

    private final PositionVoltage _anglePosition = new PositionVoltage(0).withEnableFOC(true);

    private final TorqueCurrentFOC _driveTorque = new TorqueCurrentFOC(0);

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        this.constants = moduleConstants;
        this.moduleNumber = s_moduleCount++;
        this._angleOffset = moduleConstants.EncoderOffset;
        /* Angle Encoder Config */
        _encoder = new CANCoderFactory(moduleConstants.EncoderId, SwerveConstants.canBus, SwerveConfigs.canCoder,
                "Swerve Encoder " + moduleNumber);

        _encoderSignal = (_encoder.get().getAbsolutePosition());

        /* Angle Motor Config */
        _angleMotor = new TalonFactory(moduleConstants.SteerMotorId, SwerveConstants.canBus, SwerveConfigs.angle,
                "Swerve Angle " + moduleNumber);
        resetToAbsolute();

        /* Drive Motor Config */
        _driveMotor = new TalonFactory(moduleConstants.DriveMotorId, SwerveConstants.canBus, SwerveConfigs.drive,
                "Swerve Drive " + moduleNumber);

        _anglePositionSignal = (_angleMotor.get().getPosition());
        _angleVelocitySignal = (_angleMotor.get().getVelocity());
        _drivePositionSignal = (_driveMotor.get().getPosition());
        _driveVelocitySignal = (_driveMotor.get().getVelocity());

        BaseStatusSignal.setUpdateFrequencyForAll(100, _encoderSignal, _anglePositionSignal, _angleVelocitySignal,
                _drivePositionSignal, _driveVelocitySignal);

        ParentDevice.optimizeBusUtilizationForAll(_encoder.get(), _angleMotor.get(), _driveMotor.get());

        _lastAngle = getState().angle.getRotations();
    }

    // -------------------
    // Primary entry point
    // -------------------
    public void setDesiredState(SwerveModuleState desiredState, SwerveControlMode controlMode) {
        // Optimize the swerve module's angle to prevent spinning more than 90 deg, etc.
        desiredState.optimize(getState().angle);

        double angle = (Math.abs(desiredState.speedMetersPerSecond)
                <= (SwerveConstants.maxSpeed * 0.01))
                ? _lastAngle
                : desiredState.angle.getRotations();

        // Steer the wheel
        _angleMotor.setControl(_anglePosition.withPosition(angle));

        // Then set drive speed/torque
        setSpeed(desiredState, controlMode);

        _lastAngle = angle;
    }

    private double externalTorqueDemand = 0;

    // Allow an external system (like slip-ratio PID) to set torque demand
    // if we want a separate step for your advanced traction control logic
    public void setExternalTorqueDemand(double torque) {
        externalTorqueDemand = torque;
    }

    // -----------
    // setSpeed(...)
    // -----------
    private void setSpeed(SwerveModuleState desiredState, SwerveControlMode controlMode) {
        // "ratio" helps correct direction if wheels are reversed
        double ratio = Math.cos(desiredState.angle.getRadians() - getState().angle.getRadians());
        double demandedSpeed = desiredState.speedMetersPerSecond * ratio; 

        switch (controlMode) {
            case OPEN_LOOP:
                // Map demandedSpeed to a [-1..1] voltage percent
                double voltagePercent = demandedSpeed / SwerveConstants.maxSpeed;
                _driveMotor.VoltageOut(voltagePercent * torqueScale);
                break;

            case VELOCITY:
                // Velocity-based closed-loop
                double velocityRPS = Util.MPSToRPS(demandedSpeed, SwerveConstants.wheelCircumference);
                _driveVelocity.Velocity = velocityRPS;

                _driveVelocity.FeedForward = _driveFeedForward.calculate(demandedSpeed * torqueScale);
                _driveMotor.setControl(_driveVelocity);
                break;

            case TORQUE:
                // Torque-based (current-based) closed-loop
                // The "externalTorqueDemand" might come from a slip-ratio PID or driver input.
                // Or you can derive it from demandedSpeed if you like:
                double torqueSetpoint = externalTorqueDemand == 0 ? externalTorqueDemand : computeTorqueFromSpeed(demandedSpeed); 
                // Alternatively: double torqueSetpoint = computeTorqueFromSpeed(demandedSpeed);
                
                // Command the torque/current object
                _driveTorque.Output = torqueSetpoint * torqueScale;
                // Some libraries might call this .Torque instead of .Current
                _driveMotor.setControl(_driveTorque);
                break;
        }
    }

    private double computeTorqueFromSpeed(double speedMps) {
        // A very naive approach: proportional to demanded speed
        // In reality, you'd have a slip ratio or driver input -> torque calculation
        double maxTorqueCommand = SwerveConfigs.drive.CurrentLimits.StatorCurrentLimit; // example maximum, tune to your robot
        double fraction = speedMps / SwerveConstants.maxSpeed;  // range [-1..+1]
        return fraction * maxTorqueCommand;
    }

    public Rotation2d getCANCoderAngle(boolean refresh) {
        if (refresh) {
            _encoderSignal.refresh();
        }

        return Rotation2d.fromRotations(_encoderSignal.getValueAsDouble());
    }

    public Rotation2d getCANCoderAngle() {
        return getCANCoderAngle(true);
    }

    public void resetToAbsolute() {
        double absolutePosition = getCANCoderAngle(true).getRotations() - _angleOffset;
        _angleMotor.setPosition(absolutePosition);
    }

    public SwerveModuleState getState(boolean refresh) {
        if (refresh) {
            _driveVelocitySignal.refresh();
        }
        return new SwerveModuleState(
                Util.RPSToMPS(_driveVelocitySignal.getValueAsDouble(), SwerveConstants.wheelCircumference),
                Rotation2d.fromRotations(getAngleMotorPosition(refresh)));
    }

    public SwerveModuleState getState() {
        return getState(false);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Util.rotationsToMeters(getDriveMotorPosition(), SwerveConstants.wheelCircumference),
                Rotation2d.fromRotations(getAngleMotorPosition()));
    }

    public double getDriveMotorPosition(boolean refresh) {
        if (refresh) {
            _drivePositionSignal.refresh();
            _driveVelocitySignal.refresh();
        }
        return BaseStatusSignal.getLatencyCompensatedValue(_drivePositionSignal, _driveVelocitySignal).in(Rotations);
    }
    public double getDriveMotorPosition() {
        return getDriveMotorPosition(false);
    }

    public double getAngleMotorPosition(boolean refresh) {
        if (refresh) {
            _anglePositionSignal.refresh();
            _angleVelocitySignal.refresh();
        }
        return BaseStatusSignal.getLatencyCompensatedValue(_anglePositionSignal, _angleVelocitySignal).in(Degrees);
    }

    public double getAngleMotorPosition() {
        return getAngleMotorPosition(false);
    }

    public BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[] { _anglePositionSignal, _angleVelocitySignal, _drivePositionSignal,
                _driveVelocitySignal, _encoderSignal };
    }

    public void putData() {
        SmartDashboard.putData("Swerve/Mod " + this.moduleNumber, this);
    }

    private double torqueScale = 1;
    

    public double getTorqueScale() {
        return torqueScale;
    }

    public void setTorqueScale(double torqueScale) {
        this.torqueScale = torqueScale;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Motor Angle " + moduleNumber, this::getAngleMotorPosition, null);
        builder.addDoubleProperty("Velocity " + moduleNumber, this::getDriveMotorPosition, null);
        builder.addDoubleProperty("Encoder Angle " + moduleNumber, () -> this.getCANCoderAngle().getRotations(), null);
    }
}