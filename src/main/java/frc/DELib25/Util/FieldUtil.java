package frc.DELib25.Util;

import edu.wpi.first.wpilibj.DriverStation;

public class FieldUtil {
    
    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    }

    public static boolean isBlueAllianceStrict() {
        return DriverStation.getAlliance().isPresent() && isBlueAlliance();
    }
    
    public static boolean isRedAllianceStrict() {
        return DriverStation.getAlliance().isPresent() && !isBlueAlliance();
    }
}
