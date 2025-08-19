// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gripper2Subsystem extends SubsystemBase {

	private TalonFX gripper;
	private DigitalInput beamBreak;

	/** Creates a new Gripper2Subsystem. */
	public Gripper2Subsystem() {
		gripper = new TalonFX(2);
		this.beamBreak = new DigitalInput(Constants.Gripper.beamBreakPort);
	}

	public boolean HasGamePiece() {
		return this.beamBreak.get();
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("BeamBreak", this.beamBreak.get());
	}

	public void setPercent(double Percent) {
		gripper.set(Percent);
	}

	public void disableMotors() {
		gripper.disable();
	}
}
