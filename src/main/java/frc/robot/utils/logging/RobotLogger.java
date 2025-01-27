// src/main/java/frc/robot/utils/logging/RobotLogger.java
package frc.robot.utils.logging;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.*;
import java.text.SimpleDateFormat;
import java.util.Date;

public class RobotLogger {
    private static DataLog log;
    private static RobotLogger instance;

    // Private constructor to implement singleton pattern.
    private RobotLogger() {
        // Get the DataLog instance from WPILib's DataLogManager.
        log = DataLogManager.getLog();
        // Start the DataLogManager to begin data logging.
        DataLogManager.start();
    }

    // Returns the single instance of the RobotLogger class (Singleton pattern).
    public static RobotLogger getInstance() {
        if (instance == null) {
            instance = new RobotLogger();
        }
        return instance;
    }

    // Returns a timestamp string in the format "yyyy-MM-dd HH:mm:ss.SSS".
    private String getTimestamp() {
        SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
        return sdf.format(new Date());
    }

    // Logs data to the DataLog.
    public void logData(String subsystem, String event, String details) {
        // Get the current timestamp.
        String timestamp = getTimestamp();
        // Append the log message to the DataLog, under the subsystem's "events" topic.
        // The message format is: "timestamp, event, details".
        // The timestamp is used as the entry's timestamp (0 means use current time).
        log.append(subsystem + "/events",
            timestamp + ", " + event + ", " + details,
            0);
    }
}