// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class DriveSubsystem extends SubsystemBase {

  SparkMax leftLeader = new SparkMax(2, MotorType.kBrushed);
  SparkMax leftFollower = new SparkMax(3, MotorType.kBrushed);
  SparkMax rightLeader = new SparkMax(4, MotorType.kBrushed);
  SparkMax rightFollower = new SparkMax(5, MotorType.kBrushed);
  Encoder leftEncoder = new Encoder(0, 1);
  Encoder rightEncoder = new Encoder(2, 3);
  double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI; //change this to our wheelbase dimensions (2048 or 4096 ticks, Gear Reduction)
  // kEncoderTick2Meter = Wheel circumference (m) / (Ticks per revolution * Gear Reduction)

  SparkMaxConfig globalConfig = new SparkMaxConfig();
  SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
  SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
  SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();
  
  public double getEncoderMeters() {
    return (leftEncoder.get() + -rightEncoder.get()) / 2 * kEncoderTick2Meter;
  }
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {

    globalConfig
        .smartCurrentLimit(50)
        .idleMode(IdleMode.kBrake);

    // Apply the global config and invert since it is on the opposite side
    rightLeaderConfig
        .apply(globalConfig)
        .inverted(true);

    // Apply the global config and set the leader SPARK for follower mode
    leftFollowerConfig
        .apply(globalConfig)
        .follow(leftLeader);

    // Apply the global config and set the leader SPARK for follower mode
    rightFollowerConfig
        .apply(globalConfig)
        .follow(rightLeader);
    
    //figure out replacement or removal of reset and persist modes
    /*leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    */
    
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive encoder value: ", getEncoderMeters());
    // This method will be called once per scheduler run
  }

    public void setMotors(double leftSpeed, double rightSpeed) {
      leftLeader.set(leftSpeed);
      rightLeader.set(-rightSpeed);
    }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
