// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.PersistMode;
//import com.revrobotics.spark.Sp   arkBase.BaseMode
import com.revrobotics.ResetMode;

public class DriveSubsystem extends SubsystemBase {

    private final DifferentialDrivePoseEstimator poseEstimator;

    private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

  private final SparkMax m_leftLeader = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax m_leftFollower = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax m_rightLeader = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax m_rightFollower = new SparkMax(5, MotorType.kBrushed);
  AHRS ahrs;
  PIDController turnController;


  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;


  Encoder m_leftEncoder = new Encoder(0, 1);
  Encoder m_rightEncoder = new Encoder(2, 3, true);
  double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI; // change this to our wheelbase dimensions (2048 or 4096
                                                              // ticks, Gear Reduction)
  // kEncoderTick2Meter = Wheel circumference (m) / (Ticks per revolution * Gear
  // Reduction)

  SparkMaxConfig globalConfig = new SparkMaxConfig();
  SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
  SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
  SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

  private DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);        
  poseEstimator =
                new DifferentialDrivePoseEstimator(
                        kinematics,
                        getGyroYaw(),
                        getModulePositions(),
                        new Pose2d(),
                        stateStdDevs,
                        visionStdDevs);



  public double getEncoderMeters() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance());
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
        .follow(m_leftLeader);

    // Apply the global config and set the leader SPARK for follower mode
    rightFollowerConfig
        .apply(globalConfig)
        .follow(m_rightLeader);

    // figure out replacement or removal of reset and persist modes
    m_leftLeader.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollower.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ahrs = new AHRS(NavXComType.kMXP_SPI);
    turnController = new PIDController(kP, kI, kD);

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

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

  public Command cheesyDriveCommand(DoubleSupplier fwd, DoubleSupplier rot, boolean turn_in_place) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.curvatureDrive(fwd.getAsDouble(), rot.getAsDouble(), turn_in_place))
        .withName("cheesyDrive");
  }
  
  public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), 0.7 * rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  public Command driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
            () -> {
              // Reset encoders at the start of the command
              m_leftEncoder.reset();
              m_rightEncoder.reset();
            })
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->
                Math.max(m_leftEncoder.getDistance(), m_rightEncoder.getDistance())
                    >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.stopMotor());
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Drive encoder value: ", getEncoderMeters());
    System.out.println("Right Encoder:" + m_rightEncoder.getDistance());
    System.out.println("Left Encoder:" + m_leftEncoder.getDistance());
    
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
