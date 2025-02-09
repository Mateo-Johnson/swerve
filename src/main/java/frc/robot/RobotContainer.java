package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.SwerveDriveSubsystem;

public class RobotContainer {
    private final SwerveDriveSubsystem m_driveSubsystem = new SwerveDriveSubsystem();
    private final CommandXboxController m_driverController = new CommandXboxController(0);

    public RobotContainer() {
        configureDefaultCommand(m_driveSubsystem, () -> {
            m_driveSubsystem.drive(
                -m_driverController.getLeftY(),
                m_driverController.getLeftX(),
                m_driverController.getRightX(),
                true
            );
        });
    }

    private void configureDefaultCommand(Subsystem subsystem, Runnable defaultAction) {
        subsystem.setDefaultCommand(subsystem.run(defaultAction));
    }

    private void configureBindings() {
        // Your trigger bindings here...
    }

    public Command getAutonomousCommand() {
        return null;
    }
}