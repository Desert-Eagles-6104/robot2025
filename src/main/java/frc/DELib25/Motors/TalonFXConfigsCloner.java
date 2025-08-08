package frc.DELib25.Motors;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.DifferentialConstantsConfigs;
import com.ctre.phoenix6.configs.DifferentialSensorsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;

public class TalonFXConfigsCloner {

    /**
     * Creates a clone of the TalonFXConfiguration with all parameters (not done yet)
     * @param original
     * @return
     */
    private static TalonFXConfiguration cloneTalonFXConfigs(TalonFXConfiguration original) {
        TalonFXConfiguration clone = new TalonFXConfiguration();
        /* 
        clone.FutureProofConfigs = original.FutureProofConfigs;

        public MotorOutputConfigs MotorOutput = new MotorOutputConfigs();
    
        public CurrentLimitsConfigs CurrentLimits = new CurrentLimitsConfigs();
    
        public VoltageConfigs Voltage = new VoltageConfigs();
        
        public TorqueCurrentConfigs TorqueCurrent = new TorqueCurrentConfigs();
        
        public FeedbackConfigs Feedback = new FeedbackConfigs();
        
        public DifferentialSensorsConfigs DifferentialSensors = new DifferentialSensorsConfigs();
        
        public DifferentialConstantsConfigs DifferentialConstants = new DifferentialConstantsConfigs();
        
        public OpenLoopRampsConfigs OpenLoopRamps = new OpenLoopRampsConfigs();
        
        public ClosedLoopRampsConfigs ClosedLoopRamps = new ClosedLoopRampsConfigs();
        
        public HardwareLimitSwitchConfigs HardwareLimitSwitch = new HardwareLimitSwitchConfigs();
        
        public AudioConfigs Audio = new AudioConfigs();
        
        public SoftwareLimitSwitchConfigs SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs();
        
        public MotionMagicConfigs MotionMagic = new MotionMagicConfigs();
        
        public CustomParamsConfigs CustomParams = new CustomParamsConfigs();
        
        public ClosedLoopGeneralConfigs ClosedLoopGeneral = new ClosedLoopGeneralConfigs();
        
        public Slot0Configs Slot0 = new Slot0Configs();
        
        public Slot1Configs Slot1 = new Slot1Configs();
        
        public Slot2Configs Slot2 = new Slot2Configs();
        */
        return clone;
    }  
    /**
     * Creates a clone of the TalonFXConfiguration with only essential parameters that will be most likely mutated
     * @param original
     * @return
     */
    public static TalonFXConfiguration essentialOnlyClone(TalonFXConfiguration original){
        TalonFXConfiguration clone = new TalonFXConfiguration();
        clone
            .withMotorOutput(original.MotorOutput)
            .withCurrentLimits(original.CurrentLimits)
            .withFeedback(original.Feedback)
            .withOpenLoopRamps(original.OpenLoopRamps)
            .withClosedLoopRamps(original.ClosedLoopRamps)
            .withVoltage(original.Voltage)
            .withTorqueCurrent(original.TorqueCurrent)
            .withDifferentialSensors(original.DifferentialSensors)
            .withDifferentialConstants(original.DifferentialConstants)
            .withHardwareLimitSwitch(original.HardwareLimitSwitch)
            .withAudio(original.Audio)
            .withSoftwareLimitSwitch(original.SoftwareLimitSwitch)
            .withMotionMagic(original.MotionMagic)
            .withCustomParams(original.CustomParams)
            .withClosedLoopGeneral(original.ClosedLoopGeneral)
            .withSlot0(original.Slot0)
            .withSlot1(original.Slot1)
            .withSlot2(original.Slot2)
        ;
        clone.FutureProofConfigs = original.FutureProofConfigs;
        return clone;
    }

}
