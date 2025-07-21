// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.DELib25.Sensors.BeamBreak;
import frc.robot.Constants;

public class Gripper2Subsystem extends SubsystemBase {

	TalonFX gripper;
	public BeamBreak beamBreak;
	private static boolean hasGamePiece;

	DutyCycle dutyCycleRequest;

	/** Creates a new Gripper2Subsystem. */
	public Gripper2Subsystem() {
		gripper = new TalonFX(2);
		this.beamBreak = new BeamBreak(Constants.Gripper.beamBreakPort);
	}

	public static boolean HasGamePiece() {
		return hasGamePiece;
	}

	@Override
	public void periodic() {
		this.beamBreak.update();
		hasGamePiece = this.beamBreak.get();
		SmartDashboard.putBoolean("BeamBreak", hasGamePiece);
	}

	public void setPercent(double Percent) {
		gripper.set(Percent);
	}

	public void disableMotors() {
		gripper.disable();
	}
}
