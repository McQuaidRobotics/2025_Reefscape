package igknighters.constants;

import edu.wpi.first.wpilibj.RobotController;
import igknighters.Robot;
import igknighters.util.logging.BootupLogger;
import java.util.Map;

public class RobotConfig {

  public enum RobotID {
    Mk1,
    Mk2,
    Unlabeled;

    public final String name;

    RobotID(String name) {
      this.name = name;
    }

    RobotID() {
      this.name = this.name();
    }
  }

  /** If there are duplicate serial entries the tests will fail!!!! */
  private static final Map<String, RobotID> serialToID =
      Map.of(
          "0306adcf", RobotID.Mk1,
          "0306adf3", RobotID.Mk1,
          "ffffffff", RobotID.Mk1,
          "aaaaaaaa", RobotID.Mk1,
          "03260af0", RobotID.Mk1,
          "03260abb", RobotID.Mk1,
          "0306adb6", RobotID.Mk1,
          "032b4b20", RobotID.Mk1);

  private static RobotID currentID = RobotID.Unlabeled;

  public static RobotID getRobotID() {
    if (currentID == RobotID.Unlabeled) {
      String currentSerialNum;
      if (Robot.isReal()) {
        currentSerialNum = RobotController.getSerialNumber();
        if (currentSerialNum == null) {
          throw new RuntimeException("Tried loading robot ID before advantage-kit was ready!");
        }
        currentSerialNum = currentSerialNum.toLowerCase();
      } else {
        currentSerialNum = "ffffffff";
      }
      if (serialToID.containsKey(currentSerialNum)) {
        currentID = serialToID.get(currentSerialNum);
      } else {
        throw new RuntimeException(
            "Robot ID not found, " + currentSerialNum + " not in serialToID map");
      }
      BootupLogger.bootupLog("Robot Name: " + currentID.name);
    }
    return currentID;
  }
}
