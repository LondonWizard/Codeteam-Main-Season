package frc.OscarLib.lib.HardwareDevices;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs implements LoggableInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

        @Override
        public void toLog(LogTable table) {
            table.put("Connected", connected);
            table.put("YawPosition", yawPosition);
            table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
            table.put("OdometryYawTimestamps", odometryYawTimestamps);
            table.put("OdometryYawPositions", odometryYawPositions);
        }

        @Override
        public void fromLog(LogTable table) {
            connected = table.get("Connected", connected);
            yawPosition = table.get("YawPosition", yawPosition);
            yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
            odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
            odometryYawPositions = table.get("OdometryYawPositions", odometryYawPositions);
        }
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }
}