package igknighters.util.can;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;
import wpilibExt.Tracer;

/**
 * A utility to clump CAN signals together and update them all at once. This has shown a performance
 * improvement over updating each signal individually and updating all signals per device at once.
 */
public class CANSignalManager {
  private static final HashMap<String, ArrayList<BaseStatusSignal>> signalsDatabase =
      new HashMap<>(3);

  /**
   * Registers a list of signals to be updated in the CANSignalManager
   *
   * @param canbus The name of the CAN bus to register the signals to
   * @param signals The signals to register
   */
  public static void registerSignals(String canbus, BaseStatusSignal... signals) {
    BaseStatusSignal.setUpdateFrequencyForAll(100.0, signals);
    ArrayList<BaseStatusSignal> signalsToAdd =
        signalsDatabase.computeIfAbsent(canbus, k -> new ArrayList<>(signals.length));
    for (BaseStatusSignal signal : signals) {
      signalsToAdd.add(signal);
    }
  }

  /**
   * Optimizes the bus utilization of the devices by setting the update frequency of all signals to
   *
   * @param devices
   */
  public static void registerDevices(ParentDevice... devices) {
    for (ParentDevice device : devices) {
      device.optimizeBusUtilization(4.0, 1.0);
    }
  }

  /** Refreshes all signals in the CANSignalManager, should be called once per cycle */
  public static void refreshSignals() {
    for (Entry<String, ArrayList<BaseStatusSignal>> entry : signalsDatabase.entrySet()) {
      Tracer.startTrace(entry.getKey());
      ArrayList<BaseStatusSignal> list = entry.getValue();
      if (list.size() != 0)
        BaseStatusSignal.refreshAll(list.toArray(new BaseStatusSignal[list.size()]));
      Tracer.endTrace();
    }
  }
}
