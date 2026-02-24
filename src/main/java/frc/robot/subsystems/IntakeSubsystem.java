package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType; 

public class IntakeSubsystem extends SubsystemBase {

    public final SparkMax shooter_intake = new SparkMax(6, MotorType.kBrushed);
    public final SparkMax agitator = new SparkMax(7, MotorType.kBrushed);

    public IntakeSubsystem() {
        
    }


}
