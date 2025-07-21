package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemInstantSetPosition extends InstantCommand {
	private MotorSubsystemTalon motorSubsystemTalon;
	private double position; /*homePosition/* */
	private boolean motionMagic;

	public MotorSubsystemInstantSetPosition(MotorSubsystemTalon motorSubsystemTalon , double Position) {
		this.motorSubsystemTalon = motorSubsystemTalon;
		this.position = Position;
		this.motionMagic = false;
		addRequirements(motorSubsystemTalon);
	}

	public MotorSubsystemInstantSetPosition(MotorSubsystemTalon notorSubsystemTalon, double Position, boolean motionMagic) {
		this(notorSubsystemTalon, Position);
		this.motionMagic = motionMagic;// Im pretty sure i dont need to add the requirement again, because motion magic is not a subsystem
	}

	@Override
	public void initialize() {
		if (this.motionMagic) this.motorSubsystemTalon.setMotionMagicPosition(this.position);
		else this.motorSubsystemTalon.setPosition(this.position);
	}
}
