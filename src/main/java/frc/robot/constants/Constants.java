package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemConfiguration;
import frc.DELib25.Subsystems.Vision.VisionUtil.CameraSettings;
import frc.DELib25.Util.ProjectConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	
	public final static class PigeonConfigs {
		public static final int PIGEON_ID = 44;
	}

	public static TalonFXConfiguration getDefaultTalonConfig(){
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        return talonConfig;
    }

	private static SoftwareLimitSwitchConfigs getSoftwareLimitSwitchConfigs(boolean forwardSoftLimitEnable, double forwardSoftLimitThreshold, boolean reverseSoftLimitEnable, double reverseSoftLimitThreshold,double rotationsPerPositionUnit) {
		return new SoftwareLimitSwitchConfigs()
			.withForwardSoftLimitEnable(forwardSoftLimitEnable)
			.withForwardSoftLimitThreshold(forwardSoftLimitThreshold * rotationsPerPositionUnit)
			.withReverseSoftLimitEnable(reverseSoftLimitEnable)
			.withReverseSoftLimitThreshold(reverseSoftLimitThreshold * rotationsPerPositionUnit);
	}
	

	public static TalonFXConfiguration getDefaultTalonConfig(boolean isBrake) {
		TalonFXConfiguration talonConfig = new TalonFXConfiguration();
		if (isBrake)
			talonConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

		talonConfig.MotorOutput.withDutyCycleNeutralDeadband(0.01);
		return talonConfig;
	}

	public static TalonFXConfiguration getDefaultTalonConfig(boolean isBrake,double statorCurrentLimit, double supplyCurrentLimit){
        TalonFXConfiguration talonConfig = getDefaultTalonConfig(isBrake);

        talonConfig.CurrentLimits
            .withSupplyCurrentLowerLimit(60)
            .withSupplyCurrentLowerTime(0.1)
            // There's a problem here that if i in the future enable it and then tried to retrieve the value from here it will just be the ERROR_CODE value.
            .withStatorCurrentLimitEnable(statorCurrentLimit != ProjectConstants.ERROR_CODE_INT)
            .withStatorCurrentLimit(statorCurrentLimit)
            .withSupplyCurrentLimitEnable(supplyCurrentLimit != ProjectConstants.ERROR_CODE_INT)
            .withSupplyCurrentLimit(supplyCurrentLimit);
        
        return talonConfig;
    }

  // TODO: Nothing uses this 
	public final static class Vision{
		public static final CameraSettings aprilTagCameraSettings = new CameraSettings(0.30821, 0, 0.10689, 0, 15.13, 0, false); 
		public static final CameraSettings gamePieceCameraSettings = new CameraSettings(0, 0, 0, 0, 0, 0, false); 

		public static final double cameraHeight = 0.10689;//TODO: This must be tuned to specific robot
		public static final double tragetHeight = 0.307975;//height of april tag center from the floor - reef april tag
		public static final double cameraPitch = 15.13;//limelight 3 cameraPitch
	}

	public final class Elevator {
		public static final MotorSubsystemConfiguration ElevatorConfiguration = new MotorSubsystemConfiguration() {
		{
			subsystemName = "Elevator";

			allowableError = 0.8;

			homePosition = 0.0;

			slaves = new MotorConstants[]{new MotorConstants(50,"rio",true, getDefaultTalonConfig(true,40,40))};

			master = new MotorConstants(51, "rio", true, 1 / (0.0363728 * Math.PI),
				getDefaultTalonConfig(true,40,40)
				.withMotionMagic(
					new MotionMagicConfigs()
						.withMotionMagicCruiseVelocity(26.258498391329568)
						.withMotionMagicAcceleration(52.516996782659136)
						.withMotionMagicJerk(78.7754951739887)
				).withSlot0(
					PIDContainer.toSlot0Configs(new PIDContainer(0, 1.2733, 0.060729, 0.334, 0.15, 0.0, 0.0))
				).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(10.2857142857)
				).withSoftwareLimitSwitch(
					getSoftwareLimitSwitchConfigs(true, 0.7, true, 0, 1.0 / (0.0363728 * Math.PI))
				)
			);
		}};
	}

	public final class GripperArm {
		public static final MotorSubsystemConfiguration configuration = new MotorSubsystemConfiguration() {
		{
			subsystemName = "gripperArm";

			allowableError = 3.5;

			homePosition = -89.0;

			angleOffset = 0.163;

			master = new MotorConstants(1, "rio", true, 1 /360,
				getDefaultTalonConfig(true)
				.withMotionMagic(
					new MotionMagicConfigs()
						.withMotionMagicCruiseVelocity(35000 / 360)
						.withMotionMagicAcceleration(30000 / 360)
						.withMotionMagicJerk(360 / 360)
				).withSlot0(
					PIDContainer.toSlot0Configs(new PIDContainer(0, 0, 0, 0.75, 160, 0, 0.65, GravityTypeValue.Arm_Cosine))
				).withSlot1(
					PIDContainer.toSlot1Configs(new PIDContainer(0, 0, 0, 0.75, 160, 0, 0.65, GravityTypeValue.Arm_Cosine))
				).withFeedback(
					new FeedbackConfigs().withSensorToMechanismRatio(2.28)
				).withSoftwareLimitSwitch(getSoftwareLimitSwitchConfigs(true, 36.5, true, -89.5, 1.0 / 360.0)
				).withCurrentLimits(
					new CurrentLimitsConfigs()
						.withSupplyCurrentLimit(55)
						.withSupplyCurrentLowerLimit(60)
						.withSupplyCurrentLowerTime(0.1)
						.withStatorCurrentLimitEnable(false)
						.withStatorCurrentLimit(35)
				)
			);
			
		}};
	}	

  // TODO: Nothing uses this
	public final class IntakeArm {
		public static final MotorSubsystemConfiguration configuration = new MotorSubsystemConfiguration(){
		{
			subsystemName = "intakeArm";

			allowableError = 2.0;

			homePosition = 0;
			master = new MotorConstants(4,"rio",true, 1 / 360.0,
				getDefaultTalonConfig(true)
				.withMotionMagic(
					new MotionMagicConfigs()
						.withMotionMagicCruiseVelocity(10000 / 360.0)
						.withMotionMagicAcceleration(15000 / 360.0)
						.withMotionMagicJerk(20000 / 360.0)
				).withSlot0(
					PIDContainer.toSlot0Configs(new PIDContainer(0, 0, 0, 0.65, 400.0, 0.0, 0.0))
				).withSlot1(
					PIDContainer.toSlot1Configs(new PIDContainer(0, 0, 0, 0.65, 3, 0.0, 0.0))
				).withFeedback(
					new FeedbackConfigs().withSensorToMechanismRatio(20)
				).withSoftwareLimitSwitch(
					getSoftwareLimitSwitchConfigs(true, 105.0, true, 0.0, 1.0 / 360.0)
				).withCurrentLimits(
					new CurrentLimitsConfigs()
						.withSupplyCurrentLimit(60)
						.withSupplyCurrentLowerLimit(60)
						.withSupplyCurrentLowerTime(0.1)
						.withStatorCurrentLimitEnable(false)
						.withStatorCurrentLimit(40)
				)
			);
		}};
	}

	public static final class Intake{
		public static final int motorId = 3;
		public static final int beamBreakPort = 0;
		public static final double intakeOutput = 0.6;
		public static final double OutTakeOutput = 0.7;
		public static final double Ks = 0.0; 
		public static final double Kv = 0.0; 
		public static final double Ka = 0.0;
		public static final double Kp = 0.0; 
		public static final double Ki = 0.0; 
		public static final double Kd = 0.0;
		public static final double supplyCurrentLimit = 0;
		public static final boolean SupplyCurrentLimitEnable = true;
		public static final int SensorToMechanismRatio = 0;
		public static final double DutyCycleNeutralDeadband = 0.0;
		public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;
		public static final int frequencyHz = 0;
		//Integration
		public static final double TimeToDropIntegraion = 0.5;
	}

	public static final class Gripper{
		public static final int motorId = 2;
		public static final int beamBreakPort = 2;
		public static final double intakeOutput = 0.6;
		public static final double OutTakeOutput = 0.7;
		public static final double Ks = 0.0; 
		public static final double Kv = 0.0; 
		public static final double Ka = 0.0;
		public static final double Kp = 0.0; 
		public static final double Ki = 0.0; 
		public static final double Kd = 0.0;
		public static final double supplyCurrentLimit = 20;
		public static final boolean SupplyCurrentLimitEnable = true;
		public static final int SensorToMechanismRatio = 3;
		public static final double DutyCycleNeutralDeadband = 1.0;
		public static final InvertedValue motorInverted = InvertedValue.CounterClockwise_Positive;
		public static final int frequencyHz = 0;
	}
  
}
