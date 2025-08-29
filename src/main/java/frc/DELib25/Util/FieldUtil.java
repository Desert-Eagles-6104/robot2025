package frc.DELib25.Util;

import edu.wpi.first.wpilibj.DriverStation;

public class FieldUtil {
    /**
     * Returns true if the robot is on the blue alliance, or if the alliance is not yet known, it defaults to blue.
     * @return
     */
    public static boolean isBlueAllianceOrDefault() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    }

    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() && isBlueAllianceOrDefault();
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && !isBlueAllianceOrDefault();
    }
}
