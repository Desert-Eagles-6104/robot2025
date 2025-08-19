package frc.robot.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;


public final class Constants {
 
    public static final class SysIdConstants {
        public static final Velocity<VoltageUnit> TRANSLATION_RAMP_RATE = null;
        public static final Voltage TRANSLATION_STEP_RATE = Units.Volts.of(7);
        public static final Time TRANSLATION_TIMEOUT = Units.Seconds.of(5);

        /* This is in radians per second², but SysId only supports "volts per second" */
        public static final Velocity<VoltageUnit> ROTATION_RAMP_RATE =
                Units.Volts.of(Math.PI / 6).per(Units.Second);
        /* This is in radians per second, but SysId only supports "volts" */
        public static final Voltage ROTATION_STEP_RATE = Units.Volts.of(Math.PI);
        public static final Time ROTATION_TIMEOUT = Units.Seconds.of(5);

        public static final Velocity<VoltageUnit> STEER_RAMP_RATE = null;
        public static final Voltage STEER_STEP_RATE = Units.Volts.of(7);
        public static final Time STEER_TIMEOUT = null;
    }

}
