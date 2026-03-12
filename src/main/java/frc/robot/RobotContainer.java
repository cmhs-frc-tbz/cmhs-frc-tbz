// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.FuelConstants.*;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.DRIVER_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // configure the default command, which is drive
    driveSubsystem.setDefaultCommand(driveSubsystem.cheesyDriveCommand(
        () -> -m_driverController.getLeftY(), // reverse the left joystick
        () -> m_driverController.getRightX(),
        false));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    // default command

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.\
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_driverController.leftTrigger(0.5)
        .whileTrue(fuelSubsystem.runEnd(() -> fuelSubsystem.intake(), () -> fuelSubsystem.stop()));

    m_driverController.rightTrigger(0.5).whileTrue(
        fuelSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(fuelSubsystem.launchCommand())
            .finallyDo(() -> fuelSubsystem.stop()));
    
    m_driverController.x().whileTrue(fuelSubsystem.runEnd(() -> fuelSubsystem.eject(), () -> fuelSubsystem.stop()));

    m_driverController.rightBumper().whileTrue(
        driveSubsystem.cheesyDriveCommand(
                    () -> -m_driverController.getLeftY(), // reverse the left joystick
        () -> m_driverController.getRightX()*0.8,
            true));
        // Schedule `setVelocity` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.a().whileTrue(fuelSubsystem.setVelocity(RPM.of(1000)));
    m_driverController.b().whileTrue(fuelSubsystem.setVelocity(RPM.of(6000)));
    // Schedule `set` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.x().whileTrue(fuelSubsystem.set(0.3));
    m_driverController.y().whileTrue(fuelSubsystem.set(-0.3));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An example command will be run in autonomous
  // // return Autos.exampleAuto(m_exampleSubsystem);
  // // return DriveForwardCmd();

  // }
}
