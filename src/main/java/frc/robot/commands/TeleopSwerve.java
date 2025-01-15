package frc.robot.commands;

import frc.OscarLib.lib.Swerve.SwerveConstants;
import frc.OscarLib.lib.Swerve.SwerveModule.SwerveControlMode;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        Constants.AdvancedTractionControlConstants.TractionControlLevel.OFF.apply();

    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        // Used for specific auton PID tuning
        if (RobotContainer.serializedAutoActive) {
            translationVal += Conversions.rotationsToMeters(1);
            RobotContainer.serialize();
        }

        /* Drive */
        System.out.println("Trans: " + translationVal + " | Strafe: " + strafeVal + " | Rot: " + rotationVal);

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(translationVal * SwerveConstants.maxSpeed,
                strafeVal * SwerveConstants.maxSpeed, rotationVal * SwerveConstants.maxAngularVelocity);

        boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        s_Swerve.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        chassisSpeeds,
                        isFlipped
                                ? s_Swerve.getRotation().plus(new Rotation2d(Math.PI))
                                : s_Swerve.getRotation()));
    }
}