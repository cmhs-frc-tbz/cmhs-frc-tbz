package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType; 

public class IntakeSubsystem extends SubsystemBase {

    public final SparkMax shooter_intake = new SparkMax(6, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();

    public static Mode currentMode = Mode.OFF;


    public void setMode(Mode mode){
        currentMode = mode;
    }

    
    public static enum Mode{
        OFF,
        INTAKE 1,
        SHOOT -1,
    }


    public IntakeSubsystem() {
        globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

            shooter_intake.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic(){
        switch()
    }
    // public Command turnOnIntake(){
        
    // }

}
