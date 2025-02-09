package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveDriveSubsystem;

public class SnapToAngle extends Command {
    private final SwerveDriveSubsystem m_drive;
    private final PIDController m_turningController;
    private double m_targetAngle;
    private boolean m_isFinished = false;
    private static final double ANGLE_TOLERANCE_DEGREES = 2.0;
    private static final double MAX_ROTATION_SPEED = 0.5;

    public SnapToAngle(SwerveDriveSubsystem drive) {
        m_drive = drive;
        m_turningController = new PIDController(0.5, 0.1, 0.05);
        m_turningController.enableContinuousInput(-180, 180);
        m_turningController.setTolerance(ANGLE_TOLERANCE_DEGREES);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // Find closest 90-degree increment
        double currentAngle = m_drive.getHeading();
        m_targetAngle = Math.round(currentAngle / 90.0) * 90.0;
        m_turningController.setSetpoint(m_targetAngle);
        m_isFinished = false;
    }

    @Override
    public void execute() {
        double currentAngle = m_drive.getHeading();
        double rotationSpeed = m_turningController.calculate(currentAngle);
        
        // Clamp rotation speed
        rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
        
        // Preserve current translation while rotating
        double currentX = -m_drive.getRobotRelativeSpeeds().vxMetersPerSecond;
        double currentY = m_drive.getRobotRelativeSpeeds().vyMetersPerSecond;
        
        m_drive.drive(currentX, currentY, rotationSpeed, true);

        if (m_turningController.atSetpoint()) {
            m_isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        // Don't stop modules - let them maintain their last states
        // This allows for smoother transitions when driving
    }

    // Method to manually set target angle (useful for specific alignments)
    public void setTargetAngle(double angle) {
        m_targetAngle = angle;
        m_turningController.setSetpoint(angle);
    }
}