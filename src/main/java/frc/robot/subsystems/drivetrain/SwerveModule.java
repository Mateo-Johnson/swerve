// SwerveModule.java
package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.constants.DriveConstants;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;
    private final AnalogInput absoluteEncoder;
    private final PIDController steeringPIDController;
    private final double absoluteEncoderOffsetRad;
    private final String moduleName;

    public SwerveModule(
        int driveMotorId,
        int steerMotorId,
        boolean driveMotorReversed,
        boolean steerMotorReversed,
        int absoluteEncoderId,
        double absoluteEncoderOffset,
        String moduleName
    ) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.moduleName = moduleName;

        // Configure drive motor
        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        configureSparkMax(driveMotor, driveMotorReversed);

        // Configure steering motor
        steerMotor = new SparkMax(steerMotorId, MotorType.kBrushless);
        configureSparkMax(steerMotor, steerMotorReversed);

        // Configure absolute encoder
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        // Configure PID controllers
        steeringPIDController = new PIDController(
            DriveConstants.STEERING_P,
            DriveConstants.STEERING_I,
            DriveConstants.STEERING_D
        );
        steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    private void configureSparkMax(SparkMax motor, boolean inverted) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(inverted);
        if (motor == driveMotor) {
            config.encoder
                .positionConversionFactor(Math.PI * DriveConstants.WHEEL_DIAMETER / DriveConstants.DRIVE_GEAR_RATIO)
                .velocityConversionFactor((Math.PI * DriveConstants.WHEEL_DIAMETER / DriveConstants.DRIVE_GEAR_RATIO) / 60.0);
        } else {
            config.encoder
                .positionConversionFactor(2.0 * Math.PI / DriveConstants.STEERING_GEAR_RATIO)
                .velocityConversionFactor((2.0 * Math.PI / DriveConstants.STEERING_GEAR_RATIO) / 60.0);
        }
        motor.configure(config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getVoltage() / 5.0 * 2.0 * Math.PI;
        return angle - absoluteEncoderOffsetRad;
    }

    public void resetEncoders() {
        driveMotor.getEncoder().setPosition(0);
        steerMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveMotor.getEncoder().getVelocity(),
            new Rotation2d(steerMotor.getEncoder().getPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getEncoder().getPosition(),
            new Rotation2d(steerMotor.getEncoder().getPosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the desired state to avoid spinning more than 90 degrees
        @SuppressWarnings("deprecation")
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState,
            new Rotation2d(steerMotor.getEncoder().getPosition()));

        // Apply cosine compensation
        double currentAngle = steerMotor.getEncoder().getPosition();
        double angleError = optimizedState.angle.getRadians() - currentAngle;
        double cosCompFactor = Math.cos(angleError);

        // Create new state with compensated speed
        SwerveModuleState compensatedState = new SwerveModuleState(
            optimizedState.speedMetersPerSecond * cosCompFactor,
            optimizedState.angle
        );

        // Calculate steering output
        double steeringOutput = steeringPIDController.calculate(
            currentAngle,
            compensatedState.angle.getRadians()
        );

        // Set motor outputs
        steerMotor.set(steeringOutput);
        driveMotor.getClosedLoopController()
            .setReference(compensatedState.speedMetersPerSecond,
                SparkMax.ControlType.kVelocity);

        // Log data
        SmartDashboard.putNumber(
            "Module " + moduleName + " Angle",
            Math.toDegrees(steerMotor.getEncoder().getPosition())
        );
        SmartDashboard.putNumber(
            "Module " + moduleName + " Velocity",
            driveMotor.getEncoder().getVelocity()
        );
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}