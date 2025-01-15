package frc.OscarLib.lib.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs implements LoggableInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public boolean turnEncoderConnected = false;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

        @Override
        public void toLog(LogTable table) {
            table.put("DriveConnected", driveConnected);
            table.put("DrivePositionRad", drivePositionRad);
            table.put("DriveVelocityRadPerSec", driveVelocityRadPerSec);
            table.put("DriveAppliedVolts", driveAppliedVolts);
            table.put("DriveCurrentAmps", driveCurrentAmps);

            table.put("TurnConnected", turnConnected);
            table.put("TurnEncoderConnected", turnEncoderConnected);
            table.put("TurnAbsolutePosition", turnAbsolutePosition);
            table.put("TurnPosition", turnPosition);
            table.put("TurnVelocityRadPerSec", turnVelocityRadPerSec);
            table.put("TurnAppliedVolts", turnAppliedVolts);
            table.put("TurnCurrentAmps", turnCurrentAmps);

            table.put("OdometryTimestamps", odometryTimestamps);
            table.put("OdometryDrivePositionsRad", odometryDrivePositionsRad);
            table.put("OdometryTurnPositions", odometryTurnPositions);
        }

        @Override
        public void fromLog(LogTable table) {
            driveConnected = table.get("DriveConnected", driveConnected);
            drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
            driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", driveVelocityRadPerSec);
            driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
            driveCurrentAmps = table.get("DriveCurrentAmps", driveCurrentAmps);

            turnConnected = table.get("TurnConnected", turnConnected);
            turnEncoderConnected = table.get("TurnEncoderConnected", turnEncoderConnected);
            turnAbsolutePosition = table.get("TurnAbsolutePosition", turnAbsolutePosition);
            turnPosition = table.get("TurnPosition", turnPosition);
            turnVelocityRadPerSec = table.get("TurnVelocityRadPerSec", turnVelocityRadPerSec);
            turnAppliedVolts = table.get("TurnAppliedVolts", turnAppliedVolts);
            turnCurrentAmps = table.get("TurnCurrentAmps", turnCurrentAmps);

            odometryTimestamps = table.get("OdometryTimestamps", odometryTimestamps);
            odometryDrivePositionsRad = table.get("OdometryDrivePositionsRad", odometryDrivePositionsRad);
            odometryTurnPositions = table.get("OdometryTurnPositions", odometryTurnPositions);
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {
    }

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {
    }

    /** Run the turn motor at the specified open loop value. */
    public default void setTurnOpenLoop(double output) {
    }

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec) {
    }

    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {
    }

}