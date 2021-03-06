package frc.robot.logging;

import edu.wpi.first.wpilibj.DriverStation;

public class Logger {
    private Logger() {
    }

    private static Severity severity = Severity.DEBUG;

    /**
     * Sets the minimum severity to log
     *
     * @param sev the minimum severity to log
     */
    public static void setSeverity(Severity sev) {
        severity = sev;
    }

    /**
     * Logs a message to the Driver Station
     *
     * @param sev     the severity of the message
     * @param message the message to send
     * @param t       the exception to log to stderr
     */
    public static void log(Severity sev, String message, Throwable t) {
        if (sev.level() <= severity.level()) {
            if (sev.level() <= Severity.ERROR.level())
                DriverStation.reportError(message, false);
            else if (sev.level() <= Severity.WARNING.level())
                DriverStation.reportWarning(message, false);
            else
                System.out.println(sev.toString().toUpperCase() + ": " + message);

            if (t != null) {
                String stacktrace = "\n";
                for (StackTraceElement ste : t.getStackTrace()) {
                    stacktrace += ste.toString() + "\n";
                }

                System.err.println(stacktrace);
            }
        }
    }

    /**
     * Debug message to send to the Driver Station
     *
     * @param message the message to report to the Driver Station
     */
    public static void debug(String message) {
        log(Severity.DEBUG, message, null);
    }

    /**
     * Info message to send to the Driver Station
     *
     * @param message the message to report to the Driver Station
     */
    public static void info(String message) {
        log(Severity.INFO, message, null);
    }

    /**
     * Warning message to send to the Driver Station
     *
     * @param message the message to report to the Driver Station
     */
    public static void warning(String message) {
        log(Severity.WARNING, message, null);
    }

    /**
     * Error message to send to the Driver Station
     *
     * @param message the message to report to the Driver Station
     * @param t       the exception to print to stderr
     */
    public static void error(String message, Throwable t) {
        log(Severity.ERROR, message, t);
    }

    enum Severity {
        ERROR(3), WARNING(4), INFO(6), DEBUG(7);

        private final int severity;

        /**
         * Creates a new severity with the specified level following syslog standard
         *
         * @param severity the severity from 0 to 7 (syslog standard)
         */
        private Severity(int severity) {
            this.severity = severity;
        }

        /**
         * Gets the int value of the severity
         *
         * @return the int value of the severity following syslog standards
         */
        public int level() {
            return severity;
        }
    }
}
