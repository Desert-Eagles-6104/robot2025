package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetPosition extends MotorDoubleSupplierCommand {
    protected MotorSubsystemTalon subsystemTalon;
    protected DoubleSupplier positionSupplier;

    public MotorSubsystemSetPosition(MotorSubsystemTalon motorSubsystemTalon, DoubleSupplier positionSupplier) {
        super(motorSubsystemTalon, positionSupplier);
    }

    @Override
    public void execute() {
        this.subsystemTalon.setPosition(this.positionSupplier.getAsDouble());
    }
    @Override
    public boolean isFinished() {
        return this.subsystemTalon.isAtSetpoint();
    }
}
