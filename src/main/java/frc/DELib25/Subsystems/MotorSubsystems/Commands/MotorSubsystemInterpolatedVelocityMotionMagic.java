package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemInterpolatedVelocityMotionMagic extends MotorSubsystemInterpolatedVelocity {

    public MotorSubsystemInterpolatedVelocityMotionMagic(MotorSubsystemTalon subsystemTalon, double[][] table, DoubleSupplier inputSupplier) {
        super(subsystemTalon, table, inputSupplier);
    }
    
    public MotorSubsystemInterpolatedVelocityMotionMagic(MotorSubsystemTalon subsystemTalon, String csvPath, DoubleSupplier inputSupplier) throws IOException{
        super(subsystemTalon, csvPath, inputSupplier);
    }

    @Override
    public void execute() {
        double input = this.valueSupplier.getAsDouble();
        double velocity = this.interpolator.getInterpolatedValue(input);
        this.subsystemTalon.setMotionMagicVelocity(velocity);
    }
    
}
