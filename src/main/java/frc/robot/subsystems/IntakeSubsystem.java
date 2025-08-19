// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX motor;

  private TalonFXConfiguration configuration;

  private StatusSignal<Angle> positionSignal;
  private StatusSignal<AngularVelocity> velocitySignal;
  private StatusSignal<Double> closedLoopErrorSignal;

	public IntakeSubsystem() {
		configuration = new TalonFXConfiguration();

		configuration.withMotorOutput(new MotorOutputConfigs()
			.withInverted(Constants.Intake.motorInverted)
			.withDutyCycleNeutralDeadband(Constants.Intake.DutyCycleNeutralDeadband)
		);
		
		configuration.withSlot0(new Slot0Configs()
			.withKS(Constants.Intake.Ks)
			.withKV(Constants.Intake.Kv)
			.withKA(Constants.Intake.Ka)
			.withKP(Constants.Intake.Kp)
			.withKI(Constants.Intake.Ki)
			.withKD(Constants.Intake.Kd)
		);

		configuration.withFeedback(new FeedbackConfigs()
			.withSensorToMechanismRatio(Constants.Intake.SensorToMechanismRatio)
		);

		configuration.withCurrentLimits(new CurrentLimitsConfigs()
			.withSupplyCurrentLimit(Constants.Intake.supplyCurrentLimit)
			.withSupplyCurrentLimitEnable(Constants.Intake.SupplyCurrentLimitEnable)
		);

		this.motor = new TalonFX(Constants.Intake.motorId);
		this.motor.getConfigurator().apply(configuration);

		this.positionSignal = this.motor.getPosition();
		this.velocitySignal = this.motor.getVelocity();
		this.closedLoopErrorSignal = this.motor.getClosedLoopError();

		BaseStatusSignal.setUpdateFrequencyForAll(Constants.Intake.frequencyHz ,this.positionSignal, this.velocitySignal, this.closedLoopErrorSignal);
	}


	public void disableMotors(){
		this.motor.disable();
	}

	public void setMotorPercent(double percent){
		this.motor.set(percent);
	}

}
