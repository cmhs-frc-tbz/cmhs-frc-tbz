package frc.robot.commands.autos_simple;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForwardCmd extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final double distance;
    private double encoderSetPoint;

    public DriveForwardCmd(DriveSubsystem driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        encoderSetPoint = driveSubsystem.getEncoderMeters() + distance;
        System.out.println("DriveForwardCmd started");
    }

    @Override
    public void execute() {
        driveSubsystem.setMotors(0.5, 0.5);
        System.out.println("DriveForwardCmd ended");
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


