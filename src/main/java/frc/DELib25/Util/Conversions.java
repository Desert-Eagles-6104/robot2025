package frc.DELib25.Util;

public class Conversions {
    
    /**
     * @param wheelRPS Wheel Velocity: (in Rotations per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Meters per Second)
     */
    public static double RPSToMPS(double wheelRPS, double circumference){
        double wheelMPS = wheelRPS * circumference;
        return wheelMPS;
    }

    /**
     * @param wheelMPS Wheel Velocity: (in Meters per Second)
     * @param circumference Wheel Circumference: (in Meters)
     * @return Wheel Velocity: (in Rotations per Second)
     */
    public static double MPSToRPS(double wheelMPS, double circumference){
        double wheelRPS = wheelMPS / circumference;
        return wheelRPS;
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