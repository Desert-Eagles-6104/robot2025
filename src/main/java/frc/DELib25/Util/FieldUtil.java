package frc.DELib25.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldUtil {
    /**
     * Checks whether the robot is on the blue alliance.
     * If the alliance is not yet known, it defaults to blue.
     *
     * @return true if alliance is Blue OR alliance is unknown (defaulted to Blue).
     */
    public static boolean isBlueAllianceOrDefault() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
    }

    /**
     * Returns true only if the alliance is known AND it is Blue.
     * Unlike {@link #isBlueAllianceOrDefault()}, this method will return false
     * if the alliance is not yet reported.
     *
     * @return true if explicitly on Blue alliance.
     */
    public static boolean isBlueAlliance() {
        return DriverStation.getAlliance().isPresent() && isBlueAllianceOrDefault();
    }

    /**
     * Returns true only if the alliance is known AND it is Red.
     * If the alliance is unknown, this will return false.
     *
     * @return true if explicitly on Red alliance.
     */
    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && !isBlueAllianceOrDefault();
    }

    /**
     * Gets the forward heading of the robot relative to the field,
     * based on alliance color.
     *
     * - Blue alliance forward = 0 degrees (positive X direction).
     * - Red alliance forward = 180 degrees (negative X direction).
     *
     * @return Rotation2d representing the forward direction for the current alliance.
     */
    public static Rotation2d getAllianceForwardHeading() {
        return isBlueAllianceOrDefault() ? Rotation2d.kZero : Rotation2d.k180deg;
    }

    /**
     * Returns +1 if the alliance’s forward is +X (Blue),
     * or -1 if the alliance’s forward is -X (Red).
     * Useful for flipping trajectories or joystick controls.
     *
     * @return 1 for Blue, -1 for Red.
     */
    public static int getAllianceDirectionMultiplier() {
        return isBlueAllianceOrDefault() ? 1 : -1;
    }
}
