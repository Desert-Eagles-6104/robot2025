package frc.DELib25.Subsystems.MotorSubsystems.MotorBase;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Util.PhoneixUtil;
import com.ctre.phoenix6.controls.Follower;

public class MotorSubsystemFactory {
    
    public static TalonFX createTalonFX(MotorConstants motorConstants) {
        TalonFX talon = new TalonFX(motorConstants.id, motorConstants.bus);
        talon.clearStickyFaults();
        PhoneixUtil.checkErrorAndRetry(() -> talon.getConfigurator().apply(motorConstants.getTalonFXConfig()));
        return talon;
    }

    public static TalonFX createSlaveTalon(MotorConstants slave, int masterId, boolean opposeMasterDirection) {
        TalonFX talon = createTalonFX(slave);
        PhoneixUtil.checkErrorAndRetry(() -> talon.setControl(new Follower(masterId, opposeMasterDirection)));
        return talon;
    }

    public static TalonFX[] createSlaveTalonsFX(MotorSubsystemConfiguration configuration) {
        if (configuration.slaves != null) {
            TalonFX[] slaveFX = new TalonFX[configuration.slaves.length];
            for (int i = 0; i < configuration.slaves.length; i++) {
                MotorConstants slaveConstants = configuration.slaves[i];
                slaveFX[i] = createSlaveTalon(slaveConstants, configuration.master.id,
                        configuration.master.counterClockwisePositive != slaveConstants.counterClockwisePositive);
                slaveFX[i].optimizeBusUtilization();
            }
            return slaveFX;
        }
        return null;
    }
}
