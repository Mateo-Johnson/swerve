package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.SwerveDriveSubsystem;

public class RobotContainer {
    private SendableChooser<Command> autoChooser;
    private final SwerveDriveSubsystem m_driveSubsystem = new SwerveDriveSubsystem();
    private final CommandXboxController primary = new CommandXboxController(0);
    double xSpeed;
    double ySpeed;
    double rot;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        //SLOW MODE
        if (primary.b().getAsBoolean()) {
            xSpeed = (primary.getLeftY() * 0.3);
            ySpeed = (primary.getLeftX() * 0.3);
            rot = (primary.getRightX() * 0.3);
        } else {
            xSpeed = (primary.getLeftY());
            ySpeed = (primary.getLeftX());
            rot = (primary.getRightX());
        }

        configureDefaultCommand(m_driveSubsystem, () -> {m_driveSubsystem.drive(-xSpeed, ySpeed, rot, true);});
    }

    private void configureDefaultCommand(Subsystem subsystem, Runnable defaultAction) {
        subsystem.setDefaultCommand(subsystem.run(defaultAction));
    }

    private void configureBindings() {
        // Your trigger bindings here...
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}