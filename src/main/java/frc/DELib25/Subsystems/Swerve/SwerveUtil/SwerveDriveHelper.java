// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.Swerve.SwerveUtil;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public class SwerveDriveHelper {
    
    public static enum DriveMode {
        MadTown,
        JackInTheBot,
        NewDriver,
        Raw
    }
    /**
     *  a switch case function to decide which drive mode to use
     * @param rawChassisSpeeds 
     * @param lowPower drive slowly
     * @param mode pick a drive mode
     * @return the desired drive mode
     */
    public static ChassisSpeeds updateChassisSpeeds(ChassisSpeeds rawChassisSpeeds, BooleanSupplier lowPower, DriveMode mode){
        switch(mode){
            case MadTown:
                return madTown(rawChassisSpeeds, lowPower.getAsBoolean());
                
            case JackInTheBot:
                return JackInTheBot(rawChassisSpeeds, lowPower.getAsBoolean());
            case NewDriver:
                ChassisSpeeds output = applayDeadBand(rawChassisSpeeds, 0.25);
                output.vxMetersPerSecond *=  0.5;
                output.vyMetersPerSecond *=  0.5;
                output.omegaRadiansPerSecond *= 0.5;
                return output;
            case Raw:
                return applayDeadBand(rawChassisSpeeds, 0.25);
            default:
            return madTown(rawChassisSpeeds, lowPower.getAsBoolean());
            } 
    }
    /**
     * cha
     * @param rawChassisSpeeds raw x,y and omega speeds
     * @param maxSpeed 
     * @param MaxAngularSpeed 
     * @return
     */
    public static ChassisSpeeds joystickToRobotUnits(ChassisSpeeds rawChassisSpeeds , double maxSpeed , double MaxAngularSpeed){
        ChassisSpeeds outputChassisSpeeds = new ChassisSpeeds(
        rawChassisSpeeds.vxMetersPerSecond*maxSpeed, 
        rawChassisSpeeds.vyMetersPerSecond*maxSpeed, 
        rawChassisSpeeds.omegaRadiansPerSecond*MaxAngularSpeed);
        return outputChassisSpeeds;
    }

    /**
     * helps the driver to lock on one of the four directions (north, south, east and west)
     * @param rawChassisSpeeds raw x,y and omega speeds
     * @param lowPower drive slowly
     * @return chasisSpeeds values after filtration
     */
    private static ChassisSpeeds madTown(ChassisSpeeds rawChassisSpeeds, boolean lowPower){
        //inputes
        Translation2d translationInput = new Translation2d(rawChassisSpeeds.vxMetersPerSecond, rawChassisSpeeds.vyMetersPerSecond);
        double rotationInput = rawChassisSpeeds.omegaRadiansPerSecond * 0.8; 
        double inputMagnitude = translationInput.getNorm();

        //nearest pole thrshold
        double threshold = Math.toRadians(10.0);
        if(Math.abs(translationInput.getAngle().minus(nearestPole(translationInput.getAngle())).getRadians()) < threshold) {
            translationInput = new Translation2d(inputMagnitude, nearestPole(translationInput.getAngle()));
        }

        final double translationDeadband = 0.2;
        final double rotationDeadband = 0.15;
		if(inputMagnitude < translationDeadband){
			translationInput = new Translation2d();
			inputMagnitude = 0;
		}

        /* Scale x and y by applying a power to the magnitude of the vector they create, in order
		 to make the controls less sensitive at the lower end. */
		final double power = (lowPower) ? 1.5 : 1.75;
        final double highPowerRotationScalar = 0.6;
		Rotation2d direction = translationInput.getAngle();
		double scaledMagnitude = Math.pow(inputMagnitude, power);
		translationInput = new Translation2d(direction.getCos() * scaledMagnitude, direction.getSin() * scaledMagnitude);
		
		rotationInput = (Math.abs(rotationInput) < rotationDeadband) ? 0 : rotationInput;
		rotationInput = Math.pow(Math.abs(rotationInput), 1.75) * Math.signum(rotationInput) * highPowerRotationScalar;
		
		final double translationLowPowerScalar = 0.6;
        final double rotationLowPowerScalar = 0.4;
		if(lowPower){
            translationInput = translationInput.times(translationLowPowerScalar);
			rotationInput *= rotationLowPowerScalar;
		}

        ChassisSpeeds output = new ChassisSpeeds(translationInput.getX(), translationInput.getY(),rotationInput);
        return output;
    }
    /**
     * locks on the closest pole to the inserted rotation value
     * @param rotation heading of the robot in degrees
     * @return sets the rotation to the closest pole
     */
    private static Rotation2d nearestPole(Rotation2d rotation){
        double poleSin = 0.0;
    	double poleCos = 0.0;
        double cosAngle= rotation.getCos();
        double sinAngle = rotation.getSin();
        
    	if(Math.abs(cosAngle) > Math.abs(sinAngle)){
    		poleCos = Math.signum(cosAngle);
    		poleSin = 0.0;
    	}else{
    		poleCos = 0.0;
    		poleSin = Math.signum(sinAngle);
    	}
    	return new Rotation2d(poleCos, poleSin);
    }
    /**
     * if LowPower is true sets the swerve's max speed to 60% of the max speed 
     * if LowPower is false drive with 100% of the swerve's max speed
     * @param rawChassisSpeeds raw x,y and omega speeds
     * @param lowPower drive slowly
     * @return filtered chasisSpeeds
     */
    private static ChassisSpeeds JackInTheBot(ChassisSpeeds rawChassisSpeeds, boolean lowPower){
        rawChassisSpeeds = applayDeadBand(rawChassisSpeeds, 0.25);
        ChassisSpeeds output = new ChassisSpeeds();
        if(lowPower){
            output.vxMetersPerSecond = Math.pow(rawChassisSpeeds.vxMetersPerSecond, 2) * Math.signum(rawChassisSpeeds.vxMetersPerSecond) * 0.6;
            output.vyMetersPerSecond = Math.pow(rawChassisSpeeds.vyMetersPerSecond, 2) * Math.signum(rawChassisSpeeds.vyMetersPerSecond) * 0.6;
            output.omegaRadiansPerSecond = Math.pow(rawChassisSpeeds.omegaRadiansPerSecond, 2) * Math.signum(rawChassisSpeeds.omegaRadiansPerSecond) * 0.6;
        }
        else{
            output.vxMetersPerSecond = Math.pow(rawChassisSpeeds.vxMetersPerSecond, 2) * Math.signum(rawChassisSpeeds.vxMetersPerSecond);
            output.vyMetersPerSecond = Math.pow(rawChassisSpeeds.vyMetersPerSecond, 2) * Math.signum(rawChassisSpeeds.vyMetersPerSecond);
            output.omegaRadiansPerSecond = Math.pow(rawChassisSpeeds.omegaRadiansPerSecond, 2) * Math.signum(rawChassisSpeeds.omegaRadiansPerSecond);
        }
        return output;
    }
    /**
     * applies a deadband for the chasisSpeeds to start giving value
     * @param rawChassisSpeedsraw x,y and omega speeds
     * @param deadband 
     * @return chasisSpeeds with deadband
     */
    private static ChassisSpeeds applayDeadBand(ChassisSpeeds rawChassisSpeeds, double deadband){
        Translation2d translationInput = new Translation2d(rawChassisSpeeds.vxMetersPerSecond, rawChassisSpeeds.vyMetersPerSecond);
        double rotationInput = rawChassisSpeeds.omegaRadiansPerSecond;

        translationInput = ((Math.abs(translationInput.getNorm()) < deadband) ? new Translation2d() : translationInput);
        rotationInput = (Math.abs(rotationInput) < deadband) ? 0 : rotationInput;

        return new ChassisSpeeds(translationInput.getX(), translationInput.getY(), rotationInput);
    }
}
