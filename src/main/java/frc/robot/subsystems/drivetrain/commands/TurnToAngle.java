package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveDriveSubsystem;

public class TurnToAngle extends Command {
    private final SwerveDriveSubsystem m_drive;
    private final double m_targetAngle;
    private final PIDController m_turningController;
    private boolean m_isFinished = false;

    public TurnToAngle(SwerveDriveSubsystem drive, double targetAngle) {
        m_drive = drive;
        m_targetAngle = Math.toRadians(targetAngle);
        m_turningController = new PIDController(
            0.5,  // P
            0.1,  // I
            0.05 // D
        );
        m_turningController.enableContinuousInput(-Math.PI, Math.PI);
        m_turningController.setTolerance(
            0.087,  // 5 degrees position tolerance
            0.1     // 0.1 rad/s velocity tolerance
        );
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_isFinished = false;
        m_turningController.setSetpoint(m_targetAngle);
    }

    @Override
    public void execute() {
        double rotationSpeed = m_turningController.calculate(m_drive.getHeading());
        // Clamp rotation speed to reasonable range
        rotationSpeed = MathUtil.clamp(rotationSpeed, -0.5, 0.5);
        m_drive.drive(0, 0, rotationSpeed, false);

        // Check if we're within tolerance
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
        m_drive.stopModules();
    }
}