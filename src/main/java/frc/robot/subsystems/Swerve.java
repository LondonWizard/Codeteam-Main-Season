package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Util;
import frc.OscarLib.lib.Camera.LimeLightCam;
import frc.OscarLib.lib.HardwareDevices.GyroIOPigeon2;
import frc.OscarLib.lib.Swerve.ModuleIOTalonFX;
import frc.OscarLib.lib.Swerve.SwerveDriveSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends SwerveDriveSubsystem {
    private static Swerve instance;
    private NeutralModeValue neutralMode = Constants.Swerve.driveNeutralMode;
    private SendableChooser<NeutralModeValue> neutralModeChooser = new SendableChooser<>();
    private LimeLightCam limeLight = new LimeLightCam("limelight-a", true);
    private LimeLightCam limeLight2 = new LimeLightCam("limelight-b", true);

    public NeutralModeValue getNeutralMode() {
        return neutralMode;
    }

    public void setNeutralMode(NeutralModeValue neutralMode) {
        this.neutralMode = neutralMode;
    }

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
        super(new GyroIOPigeon2());

        // poseEstimator = null;// PoseEstimator.getInstance();

        neutralModeChooser.setDefaultOption("Brake", NeutralModeValue.Brake);
        neutralModeChooser.addOption("Coast", NeutralModeValue.Coast);

    }
    /* Used by SwerveControllerCommand in Auto */

    public void zeroHeading() {
        super.resetYaw();
    }

    double distance = 0.0;
    private static final double kFieldBorderMargin = 0.5;
    private static final double kMaxVisionCorrection = 2.0; // Jump from fused pose

    private boolean poseRejected = false;
    private boolean poseRejected2 = false;
    private double xyStdDevCoeff = 0.01;
    private double rStdDevCoeff = 0.02;
    private double xyStdDev = 0.7;
    private double rStdDev = 9.0;

    public void onUpdate() {
        if (!limeLight.getEstimate().isPresent() || limeLight.getEstimate().get().tagCount <= 0
                || Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec) >= 720) {
            poseRejected = true;
        } else if (limeLight.getEstimate().get().pose.getX() < -kFieldBorderMargin
                || limeLight.getEstimate().get().pose.getX() > 52.08333 + kFieldBorderMargin
                || limeLight.getEstimate().get().pose.getY() < -kFieldBorderMargin
                || limeLight.getEstimate().get().pose.getY() > 26.583333333 + kFieldBorderMargin) {
            poseRejected = true;
        } else if (Math.hypot(getChassisSpeeds().vxMetersPerSecond,
                getChassisSpeeds().vyMetersPerSecond) > 3) {
            poseRejected = true;
        } else if (Math.abs(Math.hypot(getPose().getTranslation().getX(), getPose().getTranslation().getY())
                - Math.hypot(limeLight.getEstimate().get().pose.getX(),
                        limeLight.getEstimate().get().pose.getY())) > kMaxVisionCorrection) {
            poseRejected = true;
        } else {
            poseRejected = false;
        }

        if (!poseRejected) {
            xyStdDev = xyStdDevCoeff * Math.pow(limeLight.getEstimate().get().distToTag, 2.0)
                    / limeLight.getEstimate().get().tagCount;
            rStdDev = rStdDevCoeff * Math.pow(limeLight.getEstimate().get().distToTag, 2.0)
                    / limeLight.getEstimate().get().tagCount;
            super.addVisionMeasurement(limeLight.getEstimate().get().pose, limeLight.getEstimate().get().time,
                    VecBuilder.fill(xyStdDev, xyStdDev, rStdDev));
        }

        if (!limeLight2.getEstimate().isPresent() || limeLight2.getEstimate().get().tagCount <= 0
                || Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec) >= 720) {
            poseRejected2 = true;
        } else if (limeLight2.getEstimate().get().pose.getX() < -kFieldBorderMargin
                || limeLight2.getEstimate().get().pose.getX() > 52.08333 + kFieldBorderMargin
                || limeLight2.getEstimate().get().pose.getY() < -kFieldBorderMargin
                || limeLight2.getEstimate().get().pose.getY() > 26.583333333 + kFieldBorderMargin) {
            poseRejected2 = true;
        } else if (Math.hypot(getChassisSpeeds().vxMetersPerSecond,
                getChassisSpeeds().vyMetersPerSecond) > 3) {
            poseRejected2 = true;
        } else if (Math.abs(Math.hypot(getPose().getTranslation().getX(), getPose().getTranslation().getY())
                - Math.hypot(limeLight2.getEstimate().get().pose.getX(),
                        limeLight2.getEstimate().get().pose.getY())) > kMaxVisionCorrection) {
            poseRejected2 = true;
        } else {
            poseRejected2 = false;
        }

        if (!poseRejected2) {
            xyStdDev = xyStdDevCoeff * Math.pow(limeLight2.getEstimate().get().distToTag, 2.0)
                    / limeLight2.getEstimate().get().tagCount;
            rStdDev = rStdDevCoeff * Math.pow(limeLight2.getEstimate().get().distToTag, 2.0)
                    / limeLight2.getEstimate().get().tagCount;
            super.addVisionMeasurement(limeLight2.getEstimate().get().pose, limeLight2.getEstimate().get().time,
                    VecBuilder.fill(xyStdDev, xyStdDev, rStdDev));
        }

        limeLight.SetRobotOrientation(getRotation(), Units.radiansToDegrees(gyroInputs.yawVelocityRadPerSec));

        // Smart Dashboard numbers
        SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees());
        SmartDashboard.putData(neutralModeChooser);

        // Allow operator to change neutral mode on the fly
        if (neutralMode != neutralModeChooser.getSelected()) {
            setNeutralMode(neutralModeChooser.getSelected());
        }
        Logger.recordOutput("Pose", getPose());
    }

}