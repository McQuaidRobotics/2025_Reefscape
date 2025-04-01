package monologue;

import edu.wpi.first.util.datalog.DataLogBackgroundWriter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RuntimeType;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.time.format.DateTimeFormatter;
import java.util.Random;

public class MonoDatalogHelpers {
  private static final ZoneId utc = ZoneId.of("UTC");
  private static final DateTimeFormatter m_timeFormatter =
      DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss").withZone(utc);

  static class DriverStationState {
    public boolean dsRenamed = false;
    public boolean fmsRenamed = false;
    public int dsAttachCount = 0;
    public int fmsAttachCount = 0;
  }

  static String makeLogDir(String dir) {
    if (dir == null || !dir.isEmpty()) {
      return dir;
    }

    if (RobotBase.isReal()) {
      try {
        // prefer a mounted USB drive if one is accessible
        Path usbDir = Paths.get("/u").toRealPath();
        if (Files.isWritable(usbDir)) {
          if (!new File("/u/logs").mkdir()) {
            // ignored
          }
          return "/u/logs";
        }
      } catch (IOException ex) {
        // ignored
      }
      if (RobotBase.getRuntimeType() == RuntimeType.kRoboRIO) {
        DriverStation.reportWarning(
            "DataLogManager: Logging to RoboRIO 1 internal storage is not recommended!"
                + " Plug in a FAT32 formatted flash drive!",
            false);
      }
      if (!new File("/home/lvuser/logs").mkdir()) {
        // ignored
      }
      return "/home/lvuser/logs";
    }
    String logDir = Filesystem.getOperatingDirectory().getAbsolutePath() + "/logs";
    if (!new File(logDir).mkdir()) {
      // ignored
    }
    return logDir;
  }

  static String makeLogFilename(String filenameOverride) {
    if (filenameOverride == null || !filenameOverride.isEmpty()) {
      return filenameOverride;
    }
    Random rnd = new Random();
    StringBuilder filename = new StringBuilder();
    filename.append("FRC_TBD_");
    for (int i = 0; i < 4; i++) {
      filename.append(String.format("%04x", rnd.nextInt(0x10000)));
    }
    filename.append(".wpilog");
    return filename.toString();
  }

  static void updateLogName(DataLogBackgroundWriter log, DriverStationState dss) {
    if (!dss.dsRenamed) {
      // track DS attach
      if (DriverStation.isDSAttached()) {
        dss.dsAttachCount++;
      } else {
        dss.dsAttachCount = 0;
      }
      if (dss.dsAttachCount > 50) { // 1 second
        if (RobotController.isSystemTimeValid()) {
          LocalDateTime now = LocalDateTime.now(utc);
          log.setFilename("FRC_" + m_timeFormatter.format(now) + ".wpilog");
          dss.dsRenamed = true;
        } else {
          dss.dsAttachCount = 0; // wait a bit and try again
        }
      }
    }

    if (!dss.fmsRenamed) {
      // track FMS attach
      if (DriverStation.isFMSAttached()) {
        dss.fmsAttachCount++;
      } else {
        dss.fmsAttachCount = 0;
      }
      if (dss.fmsAttachCount > 250) { // 5 seconds
        // match info comes through TCP, so we need to double-check we've
        // actually received it
        DriverStation.MatchType matchType = DriverStation.getMatchType();
        if (matchType != DriverStation.MatchType.None) {
          // rename per match info
          char matchTypeChar =
              switch (matchType) {
                case Practice -> 'P';
                case Qualification -> 'Q';
                case Elimination -> 'E';
                default -> '_';
              };
          log.setFilename(
              "MONO_"
                  + m_timeFormatter.format(LocalDateTime.now(utc))
                  + "_"
                  + DriverStation.getEventName()
                  + "_"
                  + matchTypeChar
                  + DriverStation.getMatchNumber()
                  + ".wpilog");
          dss.fmsRenamed = true;
          dss.dsRenamed = true; // don't override FMS rename
        }
      }
    }
  }
}
