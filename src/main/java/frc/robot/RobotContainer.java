package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.OscarLib.lib.Swerve.LocalADStarAK;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

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
    private final PS4Controller driver = new PS4Controller(0);

    private final SendableChooser<Command> autoChooser;
    public static final boolean serializedAutoActive = false;

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton serialize = new JoystickButton(driver, PS4Controller.Button.kOptions.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, PS4Controller.Button.kL1.value);

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> JoystickCurvature.applyCurve(-driver.getRawAxis(translationAxis), 0.7),
                        () -> JoystickCurvature.applyCurve(-driver.getRawAxis(strafeAxis), 0.7),
                        () -> JoystickCurvature.applyCurve(driver.getRawAxis(rotationAxis), 0.7),
                        () -> robotCentric.getAsBoolean()));
        AutoBuilder.configure(s_Swerve::getPose,
                s_Swerve::resetPose,
                s_Swerve::getChassisSpeeds,
                (speeds, feedforwards) -> s_Swerve.runVelocity(speeds), new PPHolonomicDriveController( // PPHolonomicController
                        // is the built in
                        // path following
                        // controller for
                        // holonomic drive
                        // trains
                        Constants.AutoConstants.translationConstants, // Translation PID constants
                        Constants.AutoConstants.rotationConstants),
                Constants.AutoConstants.config, RobotContainer::getIsRed,
                s_Swerve);
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        Pathfinding.setPathfinder(new LocalADStarAK());

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
        serialize.onTrue(new InstantCommand(() -> serialize()));

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

    public Command getSysIDCommand() {
        var sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, // Use default config
                        (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> s_Swerve.runCharacterization(voltage.in(Volts)),
                        null, // No log consumer, since data is recorded by AdvantageKit
                        s_Swerve));
        SequentialCommandGroup sysIdCommand = new SequentialCommandGroup();
        sysIdCommand.addCommands(new Command[] { sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse) });
        return sysIdCommand;
    }
}
