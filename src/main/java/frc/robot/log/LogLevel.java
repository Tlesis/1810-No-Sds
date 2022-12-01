package frc.robot.log;

/**
 * An enum of the different log levels
 * 
 * @author Zach
 * @since 2018
 * 
 */
public enum LogLevel {
    FATAL(0), ERROR(1), STDERR(2), WARN(3), STDOUT(4), INFO(5), DEBUG(6), TRACE(7);
    /**
     * The level ID.
     */
    int levelId;

    /**
     * Gets the ID of the log level.
     * 
     * @return The level ID of the log.
     */
    public int getLevelId() {
        return levelId;
    }

    /**
     * Sets the level ID.
     * 
     * @param levelId
     */
    private LogLevel(int levelId) {
        this.levelId = levelId;
    }
}
