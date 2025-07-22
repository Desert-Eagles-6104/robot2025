package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetPosition extends Command {
    protected MotorSubsystemTalon subsystemTalon;
    protected DoubleSupplier positionSupplier;

    public MotorSubsystemSetPosition(MotorSubsystemTalon motorSubsystemTalon, DoubleSupplier positionSupplier) {
        this.subsystemTalon = motorSubsystemTalon;
        this.positionSupplier = positionSupplier;
        addRequirements(motorSubsystemTalon);
    }

    @Override
    public void execute() {
        this.subsystemTalon.setPosition(this.positionSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystemTalon.disableMotors();
    }

    @Override
    public boolean isFinished() {
        return this.subsystemTalon.isAtSetpoint();
    }
}
