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
import frc.robot.Constants;

public class GripperSubsystem extends SubsystemBase {

	private TalonFX motor;

	private boolean hasGamePiece;

	private TalonFXConfiguration configuration;

	private StatusSignal<Angle> positionSignal;
	private StatusSignal<AngularVelocity> velocitySignal;
	private StatusSignal<Double> closedLoopErrorSignal;

	public GripperSubsystem() {
		configuration = new TalonFXConfiguration();

		configuration.withMotorOutput(new MotorOutputConfigs()
			.withInverted(Constants.Gripper.motorInverted)
			.withDutyCycleNeutralDeadband(Constants.Gripper.DutyCycleNeutralDeadband)
		);

		configuration.withSlot0(new Slot0Configs()
			.withKS(Constants.Gripper.Ks)
			.withKV(Constants.Gripper.Kv)
			.withKA(Constants.Gripper.Ka)
			.withKP(Constants.Gripper.Kp)
			.withKI(Constants.Gripper.Ki)
			.withKD(Constants.Gripper.Kd)
		);

		configuration.withFeedback(new FeedbackConfigs()
			.withSensorToMechanismRatio(Constants.Gripper.SensorToMechanismRatio)
		);

		configuration.withCurrentLimits(new CurrentLimitsConfigs()
			.withSupplyCurrentLimit(Constants.Gripper.supplyCurrentLimit)
			.withSupplyCurrentLimitEnable(Constants.Gripper.SupplyCurrentLimitEnable)
		);
		this.hasGamePiece = false;

		this.motor = new TalonFX(Constants.Gripper.motorId);
		this.motor.getConfigurator().apply(configuration);

		this.positionSignal = this.motor.getPosition();
		this.velocitySignal = this.motor.getVelocity();
		this.closedLoopErrorSignal = this.motor.getClosedLoopError();

		BaseStatusSignal.setUpdateFrequencyForAll(Constants.Gripper.frequencyHz ,this.positionSignal, this.velocitySignal, this.closedLoopErrorSignal);
	}

	public void disableMotors(){
		this.motor.disable();
	}

	public boolean HasGamePiece(){
		return this.hasGamePiece;
	}

	public void setMotorPercent(double percent){
		this.motor.set(percent);
	}
}
