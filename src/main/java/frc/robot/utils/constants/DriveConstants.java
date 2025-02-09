// DriveConstants.java
package frc.robot.utils.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    // Robot physical dimensions
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.73);
    public static final double WHEEL_BASE = Units.inchesToMeters(21.73);

    // Swerve Module Positions (relative to robot center)
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0);

    // Kinematics
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION
    );

    // MK4i L3 Gear Ratios
    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEERING_GEAR_RATIO = 21.4285714;

    // Neo Motor Free Speed (RPM)
    public static final double NEO_FREE_SPEED = 5676.0;

    // Wheel diameter in meters
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);

    // Maximum speeds
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 
        (NEO_FREE_SPEED / 60.0) * WHEEL_DIAMETER * Math.PI / DRIVE_GEAR_RATIO;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 
        MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);

    // PID Constants for steering
    public static final double STEERING_P = 0.5;
    public static final double STEERING_I = 0.0;
    public static final double STEERING_D = 0.1;

    // PID Constants for drive velocity control
    public static final double DRIVE_P = 0.1;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_FF = 1.0 / MAX_VELOCITY_METERS_PER_SECOND;

    // Module specific constants
    public static final class ModuleConstants {
        // CAN IDs (adjust these based on your robot's configuration)
        public static final int FRONT_LEFT_DRIVE_ID = 1;
        public static final int FRONT_LEFT_STEER_ID = 2;
        public static final int FRONT_RIGHT_DRIVE_ID = 3;
        public static final int FRONT_RIGHT_STEER_ID = 4;
        public static final int BACK_LEFT_DRIVE_ID = 5;
        public static final int BACK_LEFT_STEER_ID = 6;
        public static final int BACK_RIGHT_DRIVE_ID = 7;
        public static final int BACK_RIGHT_STEER_ID = 8;

        // Analog encoder ports
        public static final int FRONT_LEFT_ENCODER_PORT = 0;
        public static final int FRONT_RIGHT_ENCODER_PORT = 1;
        public static final int BACK_LEFT_ENCODER_PORT = 2;
        public static final int BACK_RIGHT_ENCODER_PORT = 3;

        // Encoder offsets (in radians)
        public static final double FRONT_LEFT_ENCODER_OFFSET = 0.0;
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.0;
        public static final double BACK_LEFT_ENCODER_OFFSET = 0.0;
        public static final double BACK_RIGHT_ENCODER_OFFSET = 0.0;
    }
}