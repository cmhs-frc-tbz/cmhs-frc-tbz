// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.studica.frc.AHRS;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Function;
// import com.studica.frc.AHRS;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kLeftAuto = "Left";
  private static final String kRightAuto = "Right";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Drive train motors
  private final SparkMax left_front = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax left_rear = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax right_front = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax right_rear = new SparkMax(7, MotorType.kBrushed);
  private final SparkMax intake_motor = new SparkMax(5, MotorType.kBrushed);

  double initialDisplacementX;

  // Set all motors to coast mode

  private final XboxController joystick_0 = new XboxController(0);
  private final XboxController joystick_1 = new XboxController(1);

  private final Encoder right_encoder = new Encoder(0, 1, true, Encoder.EncodingType.k4X);
  private final Encoder left_encoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);

  AHRS ahrs;

  String autoName;

  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig rightLeaderConfig = new SparkMaxConfig();
    SparkMaxConfig leftFollowerConfig = new SparkMaxConfig();
    SparkMaxConfig rightFollowerConfig = new SparkMaxConfig();

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
        .follow(left_front);

    // Apply the global config and set the leader SPARK for follower mode
    rightFollowerConfig
        .apply(globalConfig)
        .follow(right_front);

    left_front.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    left_rear.configure(leftFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    right_front.configure(rightLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    right_rear.configure(rightFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Left Auto", kLeftAuto);
    m_chooser.addOption("Right Auto", kRightAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 240);
    camera.setFPS(30);
    right_encoder.setReverseDirection(true);
    ahrs = new AHRS(AHRS.NavXComType.kMXP_SPI);
  }

  @Override
  public void robotInit() {
    // encoder flipped

    // Define the auto choices
    String[] auto_choices = { "Default", "Left", "Right" };
    // Put the auto choices on the SmartDashboard
    SmartDashboard.putStringArray("Auto List", auto_choices);
    // Set the default auto choice
    autoName = SmartDashboard.getString("Auto Selector", "Default");

  }

  private double getYaw() {
    return ahrs.getYaw();
  }

  private void reset() {
    ahrs.reset();
  }

  public void driveForwardUsingNavX(double targetDistanceInches) {
    // Convert target distance from inches to meters (NavX uses meters)
    double targetDistanceMeters = targetDistanceInches * 0.0254; // 1 inch = 0.0254 meters
    double direction = targetDistanceInches > 0 ? 1 : -1; // Determine direction

    double initialDisplacementX = ahrs.getDisplacementX(); // Get initial displacement along the X-axis
    // Start driving forwardx
    while (Math.abs(ahrs.getDisplacementX()) - Math.abs(initialDisplacementX) < Math.abs(targetDistanceMeters)) {
      left_front.set(0.45 * direction);
      right_front.set(0.45 * direction);
      intake_motor.set(0);
    }

    // Stop the robot once the target distance is reached
    left_front.set(0);
    right_front.set(0);
    intake_motor.set(0.43);

  }

  private void resetToZeroEncoders() {
    left_encoder.reset();
    right_encoder.reset();
  }

  public void driveDistance(double distanceInches, double speed) {
    double targetCounts = distanceInches * 646.7; // Convert inches to encoder counts
    boolean isForward = distanceInches > 0; // Determine direction

    // Reset encoder
    right_encoder.reset();

    // Set direction of speed
    double driveSpeed = isForward ? speed : -speed;

    while (Math.abs(right_encoder.get()) < Math.abs(targetCounts)) {
      left_front.set(driveSpeed);
      right_front.set(driveSpeed);
      intake_motor.set(0);
    }

    // Stop the robot
    left_front.set(0);
    right_front.set(0);
    intake_motor.set(0.43);
  }

  private void stop() {
    left_front.set(0);
    right_front.set(0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    m_timer.restart();
    resetToZeroEncoders();
    reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kLeftAuto:
        // driveDistance(5, 0.1);
        driveForwardUsingNavX(-165 + 6);
        m_autoSelected = "stop";
      break;

      case kDefaultAuto:
      case kRightAuto:
        // driveDistance(5, 0.1);
        driveForwardUsingNavX(-165 + 6);
        m_autoSelected = "stop";
        break;
      case "stop":
        double count = 0;
        if (count == 1) {
          System.out.println("No Current Auto Running, or Auto is stopped.");
          count = count + 1;
        }
        break;
      default:
      driveForwardUsingNavX(-88 + 6);
      m_autoSelected = "stop";
        break;

   }
   System.out.println(String.format("Target Distance: %f", ahrs.getDisplacementX() - initialDisplacementX)); 
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    resetToZeroEncoders();
    reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double forward = joystick_0.getLeftY();
    double rotation = -joystick_0.getRightX();

    left_front.set(forward + rotation);
    right_front.set(forward - rotation);

    double negSpd = 0.4;

    if (joystick_1.getAButton()) {
      intake_motor.set(negSpd);
  } else if (joystick_1.getBButton()) {
      intake_motor.set(-negSpd);
  } else {
      intake_motor.stopMotor();
  }

    System.out.println(String.format("NavX Displacement X: %f", ahrs.getDisplacementX() - initialDisplacementX));

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
