package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType; 
import edu.wpi.first.wpilibj2.command.InstantCommand;
public class IntakeShootSubsystem extends SubsystemBase {

    public final SparkMax shooter_intake = new SparkMax(6, MotorType.kBrushless);
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    

    public IntakeShootSubsystem() {
        globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

            shooter_intake.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void set(double power){
        shooter_intake.set(power);
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
        return InstantCommand(() -> this.set(-1.0), this);
    }

}
