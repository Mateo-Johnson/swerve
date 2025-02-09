// SwerveDriveSubsystem.java
package frc.robot.subsystems.drivetrain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.constants.DriveConstants;
import frc.robot.utils.constants.DriveConstants.ModuleConstants;

public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    public final AHRS ahrs;
    private final SwerveDriveOdometry odometry;
    private final Field2d field;
    @SuppressWarnings("unused")
    private final PIDController xController;
    @SuppressWarnings("unused")
    private final PIDController yController;
    private final PIDController rotationController;
    private int periodicCounter = 0;

    public SwerveDriveSubsystem() {
        // Initialize swerve modules
        frontLeft = new SwerveModule(
            ModuleConstants.FRONT_LEFT_DRIVE_ID,
            ModuleConstants.FRONT_LEFT_STEER_ID,
            false, false,
            ModuleConstants.FRONT_LEFT_ENCODER_PORT,
            ModuleConstants.FRONT_LEFT_ENCODER_OFFSET,
            "Front Left"
        );
        
        frontRight = new SwerveModule(
            ModuleConstants.FRONT_RIGHT_DRIVE_ID,
            ModuleConstants.FRONT_RIGHT_STEER_ID,
            true, false,
            ModuleConstants.FRONT_RIGHT_ENCODER_PORT,
            ModuleConstants.FRONT_RIGHT_ENCODER_OFFSET,
            "Front Right"
        );
        
        backLeft = new SwerveModule(
            ModuleConstants.BACK_LEFT_DRIVE_ID,
            ModuleConstants.BACK_LEFT_STEER_ID,
            false, false,
            ModuleConstants.BACK_LEFT_ENCODER_PORT,
            ModuleConstants.BACK_LEFT_ENCODER_OFFSET,
            "Back Left"
        );
        
        backRight = new SwerveModule(
            ModuleConstants.BACK_RIGHT_DRIVE_ID,
            ModuleConstants.BACK_RIGHT_STEER_ID,
            true, false,
            ModuleConstants.BACK_RIGHT_ENCODER_PORT,
            ModuleConstants.BACK_RIGHT_ENCODER_OFFSET,
            "Back Right"
        );

        // Initialize AHRS
        ahrs = new AHRS(NavXComType.kMXP_SPI);

        // Initialize odometry and field visualization
        odometry = new SwerveDriveOdometry(
            DriveConstants.DRIVE_KINEMATICS,
            getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
        
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Initialize PathPlanner PID controllers
        xController = new PIDController(1.0, 0, 0);
        yController = new PIDController(1.0, 0, 0);
        rotationController = new PIDController(1.0, 0, 0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Zero heading on startup (with delay to ensure AHRS is ready)
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
                System.out.println("Error zeroing heading: " + e.getMessage());
            }
        }).start();

        // Configure AutoBuilder
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            this::getPose,                    // Robot pose supplier
            this::resetPose,                  // Method to reset odometry
            this::getRobotRelativeSpeeds,     // ChassisSpeeds supplier
            this::driveRobotRelative,         // Method to drive the robot
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
    }

    @Override
    public void periodic() {
        periodicCounter++;
        // Update odometry
        odometry.update(
            getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );

        // Update field visualization
        field.setRobotPose(getPose());

        // Log data to SmartDashboard
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        // Log AHRS status
        SmartDashboard.putBoolean("AHRS Connected", ahrs.isConnected());
        SmartDashboard.putBoolean("AHRS Calibrating", ahrs.isCalibrating());

        // Save layout periodically
        if (periodicCounter % 50 == 0) {
            SmartDashboard.putNumber("Layout Version", System.currentTimeMillis());
        }
    }

    /** Drive the robot with given speeds. */
    public void drive(double xSpeed, double ySpeed, double rotationSpeed, boolean fieldOriented) {
        // Create ChassisSpeeds object based on orientation mode
        ChassisSpeeds chassisSpeeds = fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rotationSpeed,
                getRotation2d()
            )
            : new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed);

        // Convert chassis speeds to module states
        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        // Normalize wheel speeds if any speed is greater than the maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);

        // Set each module's desired state
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    /** Sets the swerve module states directly. */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SECOND);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /** Stop all modules. */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /** Zero the gyro heading. */
    public void zeroHeading() {
        ahrs.reset();
    }

    /** Get the current heading from the AHRS. */
    public double getHeading() {
        return Math.IEEEremainder(ahrs.getAngle(), 360);
    }

    /** Get the current rotation as a Rotation2d object. */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /** Get the current pose of the robot. */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /** Reset the odometry to a specific pose. */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            },
            pose
        );
    }

    /** Reset the robot pose. */
    public void resetPose(Pose2d pose) {
        resetOdometry(pose);
    }

    /** Get the current robot-relative speeds. */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            0,
            getRotation2d()
        );
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false // Using robot-relative drive
        );
    }

    public AHRS getAHRS() {
        return ahrs;
    }

    public SwerveModule getFrontLeft() { return frontLeft; }
    public SwerveModule getFrontRight() { return frontRight; }
    public SwerveModule getBackLeft() { return backLeft; }
    public SwerveModule getBackRight() { return backRight; }
}