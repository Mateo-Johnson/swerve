package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.SwerveDriveSubsystem;

public class HoldPosition extends Command {
    private final SwerveDriveSubsystem m_drive;
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;
    private Pose2d m_targetPose;
    
    // Constants for position and rotation control
    private static final double POSITION_TOLERANCE = 0.02; // meters
    private static final double ROTATION_TOLERANCE = 2.0; // degrees
    private static final double MAX_CORRECTION_VELOCITY = 2.0; // meters per second
    private static final double MAX_ROTATION_VELOCITY = Math.PI; // radians per second

    public HoldPosition(SwerveDriveSubsystem drive) {
        m_drive = drive;
        
        // Position controllers
        m_xController = new PIDController(4.0, 0.0, 0.5);
        m_yController = new PIDController(4.0, 0.0, 0.5);
        
        // Rotation controller
        m_rotationController = new PIDController(3.0, 0.0, 0.3);
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Set tolerances
        m_xController.setTolerance(POSITION_TOLERANCE);
        m_yController.setTolerance(POSITION_TOLERANCE);
        m_rotationController.setTolerance(Math.toRadians(ROTATION_TOLERANCE));
        
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // Store current position as target
        m_targetPose = m_drive.getPose();
        
        // Set PID setpoints
        m_xController.setSetpoint(m_targetPose.getX());
        m_yController.setSetpoint(m_targetPose.getY());
        m_rotationController.setSetpoint(m_targetPose.getRotation().getRadians());
        
        // Reset PID controllers
        m_xController.reset();
        m_yController.reset();
        m_rotationController.reset();
        
        // Log initial position
        logTargetPosition();
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drive.getPose();
        
        // Calculate corrections
        double xCorrection = m_xController.calculate(currentPose.getX());
        double yCorrection = m_yController.calculate(currentPose.getY());
        double rotationCorrection = m_rotationController.calculate(
            currentPose.getRotation().getRadians());
            
        // Limit correction velocities
        xCorrection = clamp(xCorrection, -MAX_CORRECTION_VELOCITY, MAX_CORRECTION_VELOCITY);
        yCorrection = clamp(yCorrection, -MAX_CORRECTION_VELOCITY, MAX_CORRECTION_VELOCITY);
        rotationCorrection = clamp(rotationCorrection, -MAX_ROTATION_VELOCITY, MAX_ROTATION_VELOCITY);
        
        // Create chassis speeds
        ChassisSpeeds correctionSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xCorrection,
            yCorrection,
            rotationCorrection,
            currentPose.getRotation()
        );
        
        // Apply corrections
        m_drive.driveRobotRelative(correctionSpeeds);
        
        // Log current state
        logCurrentState(currentPose);
    }

    @Override
    public boolean isFinished() {
        // Command runs until interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stopModules();
    }
    
    /**
     * Set a new target pose to hold
     */
    public void setTargetPose(Pose2d newTarget) {
        m_targetPose = newTarget;
        m_xController.setSetpoint(m_targetPose.getX());
        m_yController.setSetpoint(m_targetPose.getY());
        m_rotationController.setSetpoint(m_targetPose.getRotation().getRadians());
        logTargetPosition();
    }
    
    /**
     * Get the current target pose
     */
    public Pose2d getTargetPose() {
        return m_targetPose;
    }
    
    /**
     * Check if the robot is at the target position within tolerance
     */
    public boolean isAtTarget() {
        return m_xController.atSetpoint() && 
               m_yController.atSetpoint() && 
               m_rotationController.atSetpoint();
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    private void logTargetPosition() {
        SmartDashboard.putNumber("Hold Position/Target X", m_targetPose.getX());
        SmartDashboard.putNumber("Hold Position/Target Y", m_targetPose.getY());
        SmartDashboard.putNumber("Hold Position/Target Rotation", 
            m_targetPose.getRotation().getDegrees());
    }
    
    private void logCurrentState(Pose2d currentPose) {
        SmartDashboard.putNumber("Hold Position/Current X", currentPose.getX());
        SmartDashboard.putNumber("Hold Position/Current Y", currentPose.getY());
        SmartDashboard.putNumber("Hold Position/Current Rotation", 
            currentPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Hold Position/X Error", 
            Math.abs(currentPose.getX() - m_targetPose.getX()));
        SmartDashboard.putNumber("Hold Position/Y Error", 
            Math.abs(currentPose.getY() - m_targetPose.getY()));
        SmartDashboard.putNumber("Hold Position/Rotation Error", 
            Math.abs(currentPose.getRotation().getDegrees() - 
                    m_targetPose.getRotation().getDegrees()));
        SmartDashboard.putBoolean("Hold Position/At Target", isAtTarget());
    }
}