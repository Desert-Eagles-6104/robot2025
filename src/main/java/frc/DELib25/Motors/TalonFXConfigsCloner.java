package frc.DELib25.Motors;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class TalonFXConfigsCloner {

    public static TalonFXConfiguration cloneTalonFXConfigs(TalonFXConfiguration original) {
        String serialized = original.serialize();
        TalonFXConfiguration clone = new TalonFXConfiguration();
        clone.deserialize(serialized);
        return clone;
    }

    public static Slot0Configs cloneSlot0Configs(Slot0Configs original) {
        String serialized = original.serialize();
        Slot0Configs clone = new Slot0Configs();
        clone.deserialize(serialized);
        return clone;
    }

    public static Slot1Configs cloneSlot1Configs(Slot1Configs original) {
        String serialized = original.serialize();
        Slot1Configs clone = new Slot1Configs();
        clone.deserialize(serialized);
        return clone;
    }

}
