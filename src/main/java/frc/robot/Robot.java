// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Timer;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Drive train motors
  private final SparkMax left_front = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax left_rear = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax right_front = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax right_rear = new SparkMax(7, MotorType.kBrushed);
  private final SparkMax intake_motor = new SparkMax(5, MotorType.kBrushed);

  private final XboxController joystick_0 = new XboxController(0);
  private final XboxController joystick_1 = new XboxController(1);
  String autoName;
  

  private final Timer m_timer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used for any
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
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 240);
    camera.setFPS(30);

    String[] auto_choices = {"Default", "Left", "Right"};
    SmartDashboard.putStringArray("Auto List", auto_choices);
    autoName = SmartDashboard.getString("Auto Selector", "Default"); // Thsis would make "Drive Forwards the default auto
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {


  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + autoName);
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autoName) {
      case "Left":
      if(m_timer.get() < 3)
      {
        left_front.set(-0.3);
        right_front.set(-0.3);
      }
      else
      {
        left_front.stopMotor();
        right_front.stopMotor();
      }
      if(m_timer.get() >= 3 && m_timer.get() < 3.5){}
      if(m_timer.get() >= 3.5 && m_timer.get() < 4.8)
      {
        
        intake_motor.set(0.43);
      }
      else{
        intake_motor.stopMotor();
      }
      break;

      case "Default":
      default:
        if(m_timer.get() < 2.3)
        {
          left_front.set(-0.3);
          right_front.set(-0.3);
        }
        else
        {
          left_front.stopMotor();
          right_front.stopMotor();
        }
        if(m_timer.get() >= 2.3 && m_timer.get() < 2.8){}
        if(m_timer.get() >= 2.8 && m_timer.get() < 4.0)
        {
          
          intake_motor.set(0.43);
        }
        else{
          intake_motor.stopMotor();
        }
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double forward = joystick_0.getLeftY();
    double rotation = -joystick_0.getRightX();

    left_front.set(forward + rotation);
    right_front.set(forward - rotation);

    if(joystick_0.getAButton())
    {
      intake_motor.set(0.40);
    }
    else
    {
      intake_motor.stopMotor();
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
