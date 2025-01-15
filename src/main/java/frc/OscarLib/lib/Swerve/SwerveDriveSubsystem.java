package frc.OscarLib.lib.Swerve;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.OscarLib.lib.Util;
import frc.OscarLib.lib.HardwareDevices.GyroIO;
import frc.OscarLib.lib.HardwareDevices.GyroIOPigeon2;
import frc.OscarLib.lib.HardwareDevices.PigeonFactory;
import frc.OscarLib.lib.HardwareDevices.GyroIO.GyroIOInputs;
import frc.OscarLib.lib.Swerve.SwerveModule.SwerveControlMode;
import frc.robot.Constants;
import frc.robot.Constants.AdvancedTractionControlConstants;
import frc.robot.Constants.AdvancedTractionControlConstants.TractionControlLevel;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

public abstract class SwerveDriveSubsystem extends SubsystemBase {

    private final SwerveModulePosition[] _swervePositions = new SwerveModulePosition[4];
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[4];
    private final SwerveDrivePoseEstimator _poseEstimator;

    public final double ODOMETRY_FREQUENCY = 250.0;

    public final SwerveModule[] _modules;
    protected final SwerveDriveKinematics _kinematics;
    protected final Field2d _telemetry;

    // protected final PigeonFactory _pigeon;

    protected final GyroIOPigeon2 gyroIO;
    protected final GyroIOInputs gyroInputs = new GyroIOInputs();

    protected Rotation2d yawOffset = new Rotation2d(0);
    private Rotation2d rawGyroRotation = new Rotation2d();

    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    // protected final ReadWriteLock _stateLock;
    public final Lock odometryLock = new ReentrantLock();
    protected final SwerveDriveState _cachedState;

    // private final odometryThread _odometryThread;

    protected boolean isTractionControlActive = false;

    public boolean isTractionControlActive() {
        return isTractionControlActive;
    }

    public void setTractionControlActive(boolean isTractionControlActive) {
        if (isDriftModeActive()) {
            this.isTractionControlActive = false;
        } else {
            this.isTractionControlActive = isTractionControlActive;
        }
    }

    private boolean isDriftModeActive = false;

    public boolean isDriftModeActive() {
        return isDriftModeActive;
    }

    // public void setDriftModeActive(boolean isDriftModeActive) {
    // this.isDriftModeActive = isDriftModeActive;
    // if (isDriftModeActive()) {
    // isTractionControlActive = false;
    // _modules[0]._driveMotor._motor.setNeutralMode(NeutralModeValue.Coast);
    // _modules[1]._driveMotor._motor.setNeutralMode(NeutralModeValue.Coast);
    // _modules[2]._driveMotor._motor.setNeutralMode(NeutralModeValue.Coast);
    // _modules[3]._driveMotor._motor.setNeutralMode(NeutralModeValue.Coast);
    // } else {
    // _modules[0]._driveMotor._motor.setNeutralMode(SwerveConfigs.drive.MotorOutput.NeutralMode);
    // _modules[1]._driveMotor._motor.setNeutralMode(SwerveConfigs.drive.MotorOutput.NeutralMode);
    // _modules[2]._driveMotor._motor.setNeutralMode(SwerveConfigs.drive.MotorOutput.NeutralMode);
    // _modules[3]._driveMotor._motor.setNeutralMode(SwerveConfigs.drive.MotorOutput.NeutralMode);
    // }

    // }

    protected double[] slipRatioError = { 0, 0, 0, 0 };
    protected double[] slipRatioErrorSum = { 0, 0, 0, 0 }; // For I term
    protected double[] slipRatioErrorRate = { 0, 0, 0, 0 }; // For D term
    protected double[] lastSlipRatioError = { 0, 0, 0, 0 };

    // The final torque scale for each module
    protected double[] moduleTorqueScales = { 1, 1, 1, 1 };

    // For time-based calculations (integral, derivative)
    protected double lastUpdateTime = 0.0;

    private SwerveModuleState[] swerveModuleDesiredStates = { new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState() };

    private final StructArrayPublisher<SwerveModuleState> swerveModuleDesiredStatePublisher = NetworkTableInstance
            .getDefault()
            .getStructArrayTopic("696/Swerve/DesiredStates", SwerveModuleState.struct).publish();

    private final StructArrayPublisher<SwerveModuleState> swerveModuleStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("696/Swerve/MeasuredStates", SwerveModuleState.struct).publish();

    public SwerveDriveSubsystem(GyroIOPigeon2 gyroIO) {
        // this._stateLock = new ReentrantReadWriteLock();
        this._cachedState = new SwerveDriveState();
        this._telemetry = new Field2d();

        SwerveModule frontLeft = new SwerveModule(SwerveConfigs.Mod0);
        SwerveModule frontRight = new SwerveModule(SwerveConfigs.Mod1);
        SwerveModule backLeft = new SwerveModule(SwerveConfigs.Mod2);
        SwerveModule backRight = new SwerveModule(SwerveConfigs.Mod3);
        _modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };

        this.gyroIO = gyroIO;
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        _kinematics = new SwerveDriveKinematics(SwerveConstants.modPositions);

        for (int i = 0; i < 4; ++i) {
            _swervePositions[i] = _modules[i].getPosition();
        }

        // _pigeon = new PigeonFactory(18, "rio", SwerveConfigs.pigeon, "Pigeon");

        _poseEstimator = new SwerveDrivePoseEstimator(_kinematics, getRotation(), _swervePositions,
                new Pose2d(0, 0, new Rotation2d(0)), VecBuilder.fill(0.1, 0.1, 0.01), VecBuilder.fill(0.3, 0.3, 0.6));

        resetYaw();

        lastUpdateTime = Timer.getFPGATimestamp();
        TractionControlLevel.OFF.apply();

        // _odometryThread = new odometryThread(this);
        // _odometryThread.start();
        PhoenixOdometryThread.getInstance().start();
        _telemetry.setRobotPose(new Pose2d());
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            _telemetry.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            _telemetry.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            _telemetry.getObject("path").setPoses(poses);
        });

        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
    }

    private double computePredictedWheelSpeed(ChassisSpeeds chassisSpeeds, double wheelX, double wheelY) {
        double vx = chassisSpeeds.vxMetersPerSecond - chassisSpeeds.omegaRadiansPerSecond * wheelY;
        double vy = chassisSpeeds.vyMetersPerSecond + chassisSpeeds.omegaRadiansPerSecond * wheelX;
        return Math.hypot(vx, vy);
    }

    private double calculateSlipRatio(double vPredicted, double vMeasured) {
        if (Math.abs(vPredicted) < AdvancedTractionControlConstants.kSpeedDeadband) {
            return 0.0;
        }
        return (vPredicted - vMeasured) / vPredicted;
    }

    private void updateTractionControlPID() {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        ChassisSpeeds chassisSpeeds = getChassisSpeeds();

        for (int i = 0; i < _modules.length; i++) {
            // 1. Compute predicted speed based on chassis speed + module position
            double wheelX = SwerveConstants.modPositions[i].getX();
            double wheelY = SwerveConstants.modPositions[i].getY();
            double vPredicted = computePredictedWheelSpeed(chassisSpeeds, wheelX, wheelY);

            double vMeasured = _modules[i].getState().speedMetersPerSecond;

            // 2. Slip ratio
            double slipActual = calculateSlipRatio(vPredicted, vMeasured);

            // If speeds too low, we skip slip ratio control
            if (Math.abs(vPredicted) < AdvancedTractionControlConstants.kSpeedDeadband) {
                moduleTorqueScales[i] = 1.0;
                slipRatioError[i] = 0;
                slipRatioErrorSum[i] = 0;
                slipRatioErrorRate[i] = 0;
                lastSlipRatioError[i] = 0;
                continue;
            }

            // 3. Control error: we want slipActual ~ kDesiredSlipRatio
            double error = AdvancedTractionControlConstants.kDesiredSlipRatio - slipActual;

            // PID terms
            // a) Proportional
            double pTerm = AdvancedTractionControlConstants.kP * error;

            // b) Integral
            slipRatioErrorSum[i] += error * dt;
            double iTerm = AdvancedTractionControlConstants.kI * slipRatioErrorSum[i];

            // c) Derivative
            double derivative = (error - lastSlipRatioError[i]) / dt;
            double dTerm = AdvancedTractionControlConstants.kD * derivative;
            lastSlipRatioError[i] = error;

            double pidOutput = pTerm + iTerm + dTerm;
            // If > 0, we have less slip than desired => can increase torque
            // If < 0, we have more slip than desired => reduce torque

            // 4. Convert pidOutput -> torque scale
            double newScale = 1.0 + pidOutput;
            // You may want to invert logic if you prefer negative PID to reduce torque
            // e.g., newScale = 1.0 - pidOutput; depends on how you interpret slip ratio
            // error

            // 5. Clamp
            newScale = Math.max(AdvancedTractionControlConstants.kMinTorqueScale, newScale);
            newScale = Math.min(AdvancedTractionControlConstants.kMaxTorqueScale, newScale);

            // 6. Apply
            moduleTorqueScales[i] = newScale;
            _modules[i].setTorqueScale(newScale);
        }
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = _modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = _modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return _kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = _modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native
     * units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += _modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return _poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        _poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        _poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return SwerveConstants.maxSpeed;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return SwerveConstants.maxAngularVelocity;
    }

    // @AutoLogOutput
    // public SwerveDriveState getState() {
    // try {
    // this._stateLock.readLock().lock();
    // return this._cachedState;
    // } finally {
    // this._stateLock.readLock().unlock();
    // }
    // }

    public void resetPose(Pose2d newPose) {
        resetPose(newPose);
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    // try {
    // this._stateLock.writeLock().lock();
    // _poseEstimator.resetPosition(getYaw(), _swervePositions, newPose);
    // } finally {
    // this._stateLock.writeLock().unlock();
    // }
    // }

    // public void resetPose() {
    // resetPose(new Pose2d());
    // }

    public void updateYawOffset() {
        yawOffset = getPose().getRotation().minus(getRotation());
    }

    // Called periodically -> 50 Hz
    public abstract void onUpdate();

    // @AutoLogOutput
    // public Rotation2d getYaw() {
    // return _pigeon.getYaw();
    // }

    // public Rotation2d latencyAdjustedYaw() {
    // return _pigeon.getLatencyAdjustedYaw();
    // }

    // public void zeroYaw() {
    // yawOffset = getYaw();
    // resetPose(new Pose2d(getPose().getTranslation(),
    // Rotation2d.fromDegrees(Util.getAlliance() == DriverStation.Alliance.Red ? 180
    // : 0)));
    // }

    // @AutoLogOutput
    // public ChassisSpeeds getRobotRelativeSpeeds() {
    // return getState().robotRelativeSpeeds;
    // }

    // @AutoLogOutput
    // protected SwerveModuleState[] getModuleStates() {
    // SwerveModuleState[] states = new SwerveModuleState[4];
    // for (SwerveModule mod : _modules) {
    // states[mod.moduleNumber] = mod.getState();
    // }
    // return states;
    // }

    // protected SwerveModulePosition[] getModulePositions() {
    // SwerveModulePosition[] states = new SwerveModulePosition[4];
    // for (SwerveModule mod : _modules) {
    // states[mod.moduleNumber] = mod.getPosition();
    // }
    // public void Drive(Translation2d translation, double rotation, boolean
    // fieldRelative, SwerveControlMode mode) {
    // SwerveModuleState[] swerveModuleStates = _kinematics.toSwerveModuleStates(
    // fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
    // translation.getX(),
    // translation.getY(),
    // rotation,
    // getPose().getRotation().plus(yawOffset)
    // .rotateBy(Rotation2d.fromDegrees((Util.getAlliance() == Alliance.Red ? 180 :
    // 0))))
    // : new ChassisSpeeds(
    // translation.getX(),
    // translation.getY(),
    // rotation));
    // if (isDriftModeActive()) {
    // swerveModuleStates[0].speedMetersPerSecond = 0;
    // swerveModuleStates[1].speedMetersPerSecond = 0;
    // swerveModuleStates[2].angle = new Rotation2d();
    // swerveModuleStates[2].speedMetersPerSecond = translation.getY();
    // swerveModuleStates[3].angle = new Rotation2d();
    // swerveModuleStates[3].speedMetersPerSecond = translation.getY();
    // setModuleStates(swerveModuleStates, SwerveControlMode.OPEN_LOOP);
    // } else {
    // setModuleStates(swerveModuleStates, mode);
    // }
    // } // return states;
    // }

    // public void Drive(ChassisSpeeds c) {
    // SwerveModuleState[] swerveModuleStates = _kinematics.toSwerveModuleStates(c);
    // setModuleStates(swerveModuleStates);
    // }

    // public void driveForwardVoltage(double voltage) {
    // Drive(new Translation2d(0, voltage), 0, false, SwerveControlMode.VOLTAGE);
    // }

    // public void doNothing() {
    // Drive(new ChassisSpeeds());
    // }

    public Rotation2d getYaw() {
        return gyroInputs.yawPosition;
    }

    public void resetYaw() {
        gyroIO.resetYaw();
    }

    // public void setModuleStates(SwerveModuleState[] desiredStates,
    // SwerveControlMode mode) {
    // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
    // SwerveConstants.maxSpeed);

    // for (SwerveModule mod : _modules) {
    // // if(mod._angleMotor.getID() == 1 || mod._angleMotor.getID() == 7){
    // // mod.setDesiredState(new SwerveModuleState(-5, new Rotation2d()),openLoop);
    // // }
    // // else
    // mod.setDesiredState(desiredStates[mod.moduleNumber], mode);
    // }
    // this.swerveModuleDesiredStates = desiredStates;
    // }

    // public void setModuleStates(SwerveModuleState[] desiredStates) {
    // setModuleStates(desiredStates, SwerveControlMode.VELOCITY);
    // }

    public SwerveModule[] getModules() {
        return _modules;
    }

    public double distTo(Translation2d position) {
        return getPose().getTranslation().getDistance(position);
    }

    public double distTo(Pose2d position) {
        return distTo(position.getTranslation());
    }

    public Rotation2d angleTo(Translation2d position) {
        Translation2d delta = getPose().getTranslation().minus(position);
        Rotation2d rot = Rotation2d.fromRadians(Math.atan2(delta.getY(), delta.getX()));
        return rot;
    }

    public Rotation2d angleTo(Pose2d position) {
        return angleTo(position.getTranslation());
    }

    /*
     * Don't override periodic, override onUpdate()
     * 
     */
    @Override
    public final void periodic() {
        if (DriverStation.isDisabled()) {
            this.updateYawOffset();
        }

        swerveModuleStatePublisher.set(getModuleStates());

        swerveModuleDesiredStatePublisher.set(swerveModuleDesiredStates);

        Logger.recordOutput("Slippage", _kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond
                - gyroInputs.yawVelocityRadPerSec);
        _telemetry.setRobotPose(getPose());

        SmartDashboard.putData(_telemetry);

        if (isTractionControlActive) {
            updateTractionControlPID();
        }
        onUpdate();

        Logger.recordOutput("Pose", getPose());
        Logger.recordOutput("Swerve Module States", getModuleStates());

        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : _modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : _modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = _modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = _modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = _kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            _poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected);
    }

    // public void addVisionMeasurement(Pose2d visionPose, double visionTimestamp,
    // Vector<N3> stdDeviations) {
    // try {
    // this._stateLock.writeLock().lock();

    // _poseEstimator.addVisionMeasurement(visionPose, visionTimestamp,
    // stdDeviations);
    // } finally {
    // this._stateLock.writeLock().unlock();
    // }
    // }

    public String getName() {
        return "Drive";
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModulePositions()[i].angle;
        }
        _kinematics.resetHeadings(headings);
        stop();
    }

    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = _kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.maxSpeed);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            _modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            _modules[i].runCharacterization(output);
        }
    }

    // class odometryThread extends Thread {
    // private SwerveDriveSubsystem this0;

    // private BaseStatusSignal[] _allSignals;

    // odometryThread(SwerveDriveSubsystem this0) {
    // this.this0 = this0;

    // this.setDaemon(true);
    // this.setPriority(MIN_PRIORITY);
    // }

    // public void run() {
    // _allSignals = new BaseStatusSignal[this0._modules.length *
    // SwerveModule.SIGNAL_COUNT + 2];
    // for (SwerveModule mod : this0._modules) {

    // BaseStatusSignal[] signals = mod.getSignals();

    // for (int i = 0; i < SwerveModule.SIGNAL_COUNT; i++) {
    // _allSignals[mod.moduleNumber * SwerveModule.SIGNAL_COUNT + i] = signals[i];
    // }
    // }
    // _allSignals[_allSignals.length - 2] = this0._pigeon._yawSignal;
    // _allSignals[_allSignals.length - 1] = this0._pigeon._yawVelocitySignal;

    // BaseStatusSignal.setUpdateFrequencyForAll(250, _allSignals);

    // while (true) {
    // try {
    // BaseStatusSignal.refreshAll(_allSignals);

    // this.this0._stateLock.writeLock().lock();

    // for (int i = 0; i < 4; ++i) {
    // _swervePositions[i] = _modules[i].getPosition();
    // }

    // ChassisSpeeds speeds = _kinematics.toChassisSpeeds(getModuleStates());

    // double time = System.currentTimeMillis();

    // Pose2d newPose = _poseEstimator.update(latencyAdjustedYaw(),
    // _swervePositions);

    // this.this0._cachedState.update(
    // newPose,
    // speeds,
    // time);
    // } finally {
    // this.this0._stateLock.writeLock().unlock();
    // }
    // Timer.delay(1.0 / 1000.0); // Limits To 1000 Hz
    // }
    // }
    // }
}
