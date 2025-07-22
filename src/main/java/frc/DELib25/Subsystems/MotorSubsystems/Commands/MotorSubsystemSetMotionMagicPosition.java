package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetMotionMagicPosition extends MotorSubsystemSetPosition {

    public MotorSubsystemSetMotionMagicPosition(MotorSubsystemTalon subsystemTalon, DoubleSupplier positionSupplier) {
        super(subsystemTalon, positionSupplier);
    }

    @Override
    public void execute() {
        this.subsystemTalon.setMotionMagicPosition(this.positionSupplier.getAsDouble());
    }
}
