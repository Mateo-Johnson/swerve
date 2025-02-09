package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveDriveSubsystem;

public class MoveDistance extends Command {
    private final SwerveDriveSubsystem m_drive;
    private final double m_targetDistance;
    private final PIDController m_distanceController;
    private boolean m_isFinished = false;
    private double initialPoseY;

    public MoveDistance(SwerveDriveSubsystem drive, double targetDistance) {
        m_drive = drive;
        m_targetDistance = targetDistance;
        // Create PID controller with gains
        m_distanceController = new PIDController(
            1.0,  // P
            0.2,  // I
            0.1   // D
        );
        
        // Set tolerance values
        m_distanceController.setTolerance(
            0.05,  // Position tolerance (5cm)
            Double.POSITIVE_INFINITY  // No velocity tolerance needed for position control
        );
        
        addRequirements(m_drive);
    }
    
    @Override
    public void initialize() {
        m_isFinished = false;
        initialPoseY = m_drive.getPose().getY();
        m_distanceController.setSetpoint(initialPoseY + m_targetDistance);
    }

    @Override
    public void execute() {
        double currentPoseY = m_drive.getPose().getY();
        double distanceError = m_distanceController.calculate(currentPoseY);
        
        // Limit speed based on distance remaining
        double speed = Math.abs(distanceError);
        speed = Math.min(speed, 1.0); // Maximum speed
        
        // Apply signs correctly
        speed = distanceError > 0 ? speed : -speed;
        
        m_drive.drive(0, speed, 0, false);
        
        // Check if we're within tolerance
        if (m_distanceController.atSetpoint()) {
            m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stopModules();
    }
}