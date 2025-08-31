package frc.DELib25.Util;

public class Conversions {
    
    /**
     * @param wheelRotPerSec Wheel velocity (rotations per second).
     * @param circumference Wheel circumference (meters).
     * @return Wheel linear velocity (meters per second).
     */
    public static double RotPerSecToMPS(double wheelRotPerSec, double circumference) {
        return wheelRotPerSec * circumference;
    }

    /**
     * @param wheelMPS Wheel linear velocity (meters per second).
     * @param circumference Wheel circumference (meters).
     * @return Wheel velocity (rotations per second).
     */
    public static double MPSToRotPerSec(double wheelMPS, double circumference) {
        return wheelMPS / circumference;
    }

    /**
     * Convert linear speed at a module into angular speed of the robot.
     *
     * @param wheelMPS Wheel linear velocity (m/s).
     * @param radiusMeters Distance from robot center to module (m).
     * @return Robot angular velocity (radians per second).
     */
    public static double MPSToRadPerSec(double wheelMPS, double radiusMeters) {
        return wheelMPS / radiusMeters;
    }

    /**
     * Convert angular robot speed into the linear speed a module must travel.
     *
     * @param radPerSec Robot angular velocity (rad/s).
     * @param radiusMeters Distance from robot center to module (m).
     * @return Module linear velocity (m/s).
     */
    public static double RadPerSecToMPS(double radPerSec, double radiusMeters) {
        return radPerSec * radiusMeters;
    }


    /**
     * @param wheelRotations Wheel Position: (in Rotations)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Distance: (in Meters)
     */
    public static double rotationsToMeters(double wheelRotations, double circumference){
        double wheelMeters = wheelRotations * circumference;
        return wheelMeters;
    }

    public static double degreesToRadians(double degrees){
        double Radians = Math.toRadians(degrees);
        return Radians;
    }

    /**
     * @param wheelMeters Wheel Distance: (in Meters)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Position: (in Rotations)
     */
    public static double metersToRotations(double wheelMeters, double circumference){
        double wheelRotations = wheelMeters / circumference;
        return wheelRotations;
    }
    /**
     * @param seconds Time: (in Seconds)
     * @return Time: (in Milliseconds)
     */
    public static double millToSeconds(double milliSeconds){
        double seconds = milliSeconds / 1000.0;
        return seconds;
    }
    /**
     * @param seconds Time: (in Seconds)
     * @return Time: (in Milliseconds)
     */
    public static double ticksToSeconds(double ticks, double ticksPerSecond){
        double seconds = ticks / ticksPerSecond;
        return seconds;
    }
}