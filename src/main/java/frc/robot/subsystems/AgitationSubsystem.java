package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType; 

public class AgitationSubsystem extends SubsystemBase {

    public final SparkMax agitator = new SparkMax(6, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    



    public AgitationSubsystem() {
        globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

            agitator.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void set(double power){
        agitator.set(power);
    }

    public Command runIntakeCommand() {
        // implicitly requires `this`
        return this.startEnd(() -> this.set(1.0), null);
    }

    public Command stopCommand(){
        return this.startEnd(() -> this.set(0.0), null);
    }

    public Command runShootCommand() {
        // implicitly requires `this`
        return this.startEnd(() -> this.set(-1.0), null);
    }


}

