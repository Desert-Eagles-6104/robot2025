// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.DELib25.Motors;

// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkFlex;

// /** Add your docs here. */
// public class SparkFlexFactory {
//     public static CANSparkFlex createSparkFlex(MotorConstants motorConstants, boolean useDefaultConfig){
//         CANSparkFlex spark = createSparkFlex(motorConstants.id);
//         if(useDefaultConfig){
//             spark.setCANMaxRetries(5);
//             spark.setIdleMode(MotorConstants.toIdleMode(motorConstants.isBrake));
//             spark.setInverted(motorConstants.CounterClockwisePositive);
//             spark.enableVoltageCompensation(12);
//             spark.setSmartCurrentLimit(40);
//             spark.burnFlash();
//         }
//         return spark;
//     }

//     public static CANSparkFlex createSlaveSparkFlex(CANSparkFlex master, int slaveId, boolean opposeMasterDirection){
//         CANSparkFlex spark = createSparkFlex(slaveId);
//         spark.follow(master, opposeMasterDirection);
//         spark.burnFlash();
//         return spark;
//     }
    
//     public static CANSparkFlex createSparkFlex(int id){
//         CANSparkFlex spark = new CANSparkFlex(id, MotorType.kBrushless);
//         spark.restoreFactoryDefaults();
//         spark.clearFaults();
//         return spark;
//     }
// }
