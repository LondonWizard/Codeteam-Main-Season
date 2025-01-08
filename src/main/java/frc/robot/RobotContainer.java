package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    private final SendableChooser<Command> autoChooser;
    public static final boolean serializedAutoActive = false;

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kOptions.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton shootButton = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    private final JoystickButton intakeButton = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
    private final JoystickButton goToSpeakerButton = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    private final JoystickButton outtakeButton = new JoystickButton(driver, PS4Controller.Button.kL2.value);

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));
        AutoBuilder.configure(s_Swerve::getPose,
                s_Swerve::resetPose,
                s_Swerve::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> s_Swerve.Drive(speeds), new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                Constants.AutoConstants.translationConstants, // Translation PID constants
                Constants.AutoConstants.rotationConstants), Constants.AutoConstants.config, RobotContainer::getIsRed,
                s_Swerve);
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");

        // Configure the button bindings
        configureButtonBindings();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> serialize()));
        goToSpeakerButton.whileTrue(new InstantCommand(() -> AutoBuilder
                .pathfindToPoseFlipped(new Pose2d(new Translation2d(1.49, 5.55), Rotation2d.fromDegrees(0)),
                        Constants.AutoConstants.GLOBAL_CONSTRAINTS)));

    }

    public static boolean getIsRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public static void serialize() {
        String motorSerialString = "4leXx564cg";
        Integer.parseInt(motorSerialString);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
