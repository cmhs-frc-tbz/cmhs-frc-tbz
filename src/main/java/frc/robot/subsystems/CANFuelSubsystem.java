// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.SmartMechanism;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;



public class CANFuelSubsystem extends SubsystemBase {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller =  new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  // Feedforward Constants
  .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
  // You could also use .withGearing(12) which does the same thing.
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController sparkSmartMotorController = new SparkWrapper(intakeLauncherRoller, DCMotor.getNEO(1), smcConfig);
  
  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(sparkSmartMotorController)
  // Diameter of the flywheel.
    .withMOI(MomentOfInertia.ofBaseUnits(0.0400560768, KilogramSquareMeters))

    // Maximum speed of the shooter.
  .withUpperSoftLimit(RPM.of(5600))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)

;


  private FlyWheel shooter = new FlyWheel(shooterConfig);



  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

    // put default values for various fuel operations onto the dashboard
    // all methods in this subsystem pull their values from the dashbaord to allow
    // you to tune the values easily, and then replace the values in Constants.java
    // with your new values. For more information, see the Software Guide.
    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.inverted(true);
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(false);
     launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {return shooter.getSpeed();}

    public Command sysId() { 
      return shooter.sysId(
        Volts.of(7),              // Step voltage for dynamic test
        Volts.of(2).per(Second),  // Ramp rate for quasistatic test
        Seconds.of(4)             // Maximum test duration
      );
    }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {return shooter.run(speed);}
  
  /**
   * Set the shooter velocity setpoint.
   *
   * @param speed Speed to set
   */
  public void setVelocitySetpoint(AngularVelocity speed) {shooter.setMechanismVelocitySetpoint(speed);}
  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return shooter.set(dutyCycle);}



  // A method to set the rollers to values for intaking
  public void intake() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feederRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(-1 * SmartDashboard.getNumber("Intaking launcher roller value", INTAKING_INTAKE_VOLTAGE));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feederRoller.setVoltage(SmartDashboard.getNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A method to stop the rollers
  public void stop() {
    feederRoller.set(0);
    intakeLauncherRoller.set(0);
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feederRoller
        .setVoltage(SmartDashboard.getNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE));
    intakeLauncherRoller
        .setVoltage(SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE));
  }

  // A command factory to turn the spinUp method into a command that requires this
  // subsystem
  public Command spinUpCommand() {
    return this.run(() -> spinUp());
  }

  // A command factory to turn the launch method into a command that requires this
  // subsystem
  public Command launchCommand() {
    return this.run(() -> launch());
  }

  public void periodic() {
    // This method will be called once per scheduler run
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooter.simIterate();
  }
}
