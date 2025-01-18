package frc.robot.Constants;

public class Hang_Constants {
    public static final class Pneumatics {
        // Port for the REV Pneumatics Hub compressor (assume default port 0)
        public static final int COMPRESSOR_PORT = 0;

        // Channels for the solenoids connected to the REV Pneumatics Hub
        public static final int SOLENOID_1_CHANNEL = 1;
        public static final int SOLENOID_2_CHANNEL = 2;

        // Pressure thresholds for the compressor's analog control
        public static final int MIN_PRESSURE = 0; // Start compressor below this PSI
        public static final int MAX_PRESSURE = 120; // Stop compressor above this PSI
    }
}
