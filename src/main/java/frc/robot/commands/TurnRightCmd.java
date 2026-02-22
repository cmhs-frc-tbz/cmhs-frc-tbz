package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class TurnRightCmd extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final double distance;
    private final double speed;
    private double encoderSetPoint;

    public TurnRightCmd(DriveSubsystem driveSubsystem, double distance, double speed) {
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        this.speed = speed;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        encoderSetPoint = driveSubsystem.getEncoderMeters() + distance;
        System.out.println("TurnLeftCmd started");
    }

    @Override
    public void execute() {
        driveSubsystem.setMotors(speed, -speed);
        System.out.println("TurnLeftCmd ended");
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (driveSubsystem.getEncoderMeters() > encoderSetPoint) {
            return true;
        } else {
            return false;
        }
    }
}


