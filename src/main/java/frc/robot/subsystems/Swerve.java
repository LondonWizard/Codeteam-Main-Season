package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Util;
import frc.OscarLib.lib.Camera.LimeLightCam;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Swerve extends SwerveDriveSubsystem {
    private static Swerve instance;
    private NeutralModeValue neutralMode = Constants.Swerve.driveNeutralMode;
    private SendableChooser<NeutralModeValue> neutralModeChooser = new SendableChooser<>();
    private LimeLightCam limeLight = new LimeLightCam("limelight-a");
    // private PoseEstimator poseEstimator;

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
        // poseEstimator = null;// PoseEstimator.getInstance();

        neutralModeChooser.setDefaultOption("Brake", NeutralModeValue.Brake);
        neutralModeChooser.addOption("Coast", NeutralModeValue.Coast);

    }
    /* Used by SwerveControllerCommand in Auto */

    public void zeroHeading() {
        super.zeroYaw();
    }

    double distance = 0.0;

    public void onUpdate() {
        if (limeLight.getEstimate().isPresent()) {
            super.addVisionMeasurement(limeLight.getEstimate().get().pose, limeLight.getEstimate().get().time,
                    VecBuilder.fill(.3, .3, .7));
        }

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