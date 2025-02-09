package frc.robot.utils.constants;

    // Module specific constants
    public final class ModuleConstants {
        // CAN IDs and Analog encoder ports

        //FRONT RIGHT
        public static final int FRONT_RIGHT_DRIVE_ID = 3;
        public static final int FRONT_RIGHT_STEER_ID = 4;
        public static final int FRONT_RIGHT_ENCODER_PORT = 1;

        //FRONT LEFT
        public static final int FRONT_LEFT_DRIVE_ID = 1;
        public static final int FRONT_LEFT_STEER_ID = 2;
        public static final int FRONT_LEFT_ENCODER_PORT = 0;

        //BACK RIGHT
        public static final int BACK_RIGHT_DRIVE_ID = 7;
        public static final int BACK_RIGHT_STEER_ID = 8;
        public static final int BACK_RIGHT_ENCODER_PORT = 3;

        //BACK LEFT
        public static final int BACK_LEFT_DRIVE_ID = 5;
        public static final int BACK_LEFT_STEER_ID = 6;
        public static final int BACK_LEFT_ENCODER_PORT = 2;

        // Encoder offsets (in radians)
        public static final double FRONT_LEFT_ENCODER_OFFSET = 0.0;
        public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.0;
        public static final double BACK_LEFT_ENCODER_OFFSET = 0.0;
        public static final double BACK_RIGHT_ENCODER_OFFSET = 0.0;
    }
