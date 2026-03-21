package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

public final class RobotLog {
  private static final Logger LOGGER = Logger.getLogger("Robot");
  private static boolean initialized = false;

  private RobotLog() {}

  public static void init() {
    if (initialized) {
      return;
    }
    initialized = true;

    try {
      Path logDir = Paths.get("/home/lvuser");
      Files.createDirectories(logDir);
      Path logPath = logDir.resolve("robot-errors.log");
      FileHandler handler = new FileHandler(logPath.toString(), true);
      handler.setFormatter(new SimpleFormatter());
      handler.setLevel(Level.ALL);
      LOGGER.addHandler(handler);
      LOGGER.setUseParentHandlers(true);
      LOGGER.setLevel(Level.ALL);
      LOGGER.info("Robot logging initialized at " + logPath);
    } catch (Exception ex) {
      LOGGER.log(Level.SEVERE, "Failed to initialize robot log file", ex);
    }
  }

  public static void installUncaughtExceptionHandler() {
    Thread.setDefaultUncaughtExceptionHandler(
        (thread, throwable) -> logException("Uncaught exception in " + thread.getName(), throwable));
  }

  public static void logException(String message, Throwable throwable) {
    if (throwable == null) {
      LOGGER.severe(message + " (null throwable)");
      return;
    }
    StringWriter sw = new StringWriter();
    throwable.printStackTrace(new PrintWriter(sw));
    LOGGER.severe(message + "\n" + sw.toString());
  }
}
