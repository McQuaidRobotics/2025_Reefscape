package igknighters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.wpilibj.DriverStation;
import igknighters.util.logging.CtreStructs;
import igknighters.util.logging.CtreStructs.TalonFXSummary;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.VarHandle;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;
import java.util.function.Supplier;
import monologue.Logged;
import monologue.Monologue;
import wpilibExt.Tracer;

public class DeviceManager {
  private static final VarHandle DEVICE_HEARTBEAT_HANDLE, DEVICE_RESET_HANDLE;

  static {
    try {
      if (Robot.isSimulation()) {
        DEVICE_HEARTBEAT_HANDLE = null;
        DEVICE_RESET_HANDLE = null;
      } else {
        var privateLookup =
            MethodHandles.privateLookupIn(ParentDevice.class, MethodHandles.lookup());
        DEVICE_HEARTBEAT_HANDLE =
            privateLookup.findVarHandle(ParentDevice.class, "_compliancy", StatusSignal.class);
        DEVICE_RESET_HANDLE =
            privateLookup.findVarHandle(ParentDevice.class, "_resetSignal", StatusSignal.class);
      }
    } catch (NoSuchFieldException | IllegalAccessException e) {
      throw new RuntimeException(e);
    }
  }

  private static BaseStatusSignal getHeartBeatSignal(ParentDevice device) {
    try {
      StatusSignal<Integer> sig = (StatusSignal<Integer>) DEVICE_HEARTBEAT_HANDLE.get(device);
      return sig.clone();
    } catch (Throwable e) {
      throw new RuntimeException(e);
    }
  }

  private static BaseStatusSignal getResetSignal(ParentDevice device) {
    try {
      StatusSignal<Integer> sig = (StatusSignal<Integer>) DEVICE_RESET_HANDLE.get(device);
      return sig.clone();
    } catch (Throwable e) {
      throw new RuntimeException(e);
    }
  }

  /** A CAN retry error. */
  public static class CANRetryError extends RuntimeException {
    private CANRetryError(String action) {
      super("Can retry limit exceeded for \"" + action + "\", marked fatal!!!!");
    }
  }

  /**
   * Given a CTRE Status Code will retry a given amount of times and return true if the code ever
   * returns ok
   *
   * @param statusCodeSup The status code.
   * @param action The action to retry.
   * @param retryLimit The retry limit.
   * @return Whether the status code is ok.
   */
  public boolean retryStatusCode(Supplier<StatusCode> statusCodeSup, String action, int retryLimit) {
    for (int i = 0; i < retryLimit; i++) {
      if (statusCodeSup.get().isOK()) return true;
    }
    // DriverStation.reportWarning(
    //     "Status Code " + statusCodeSup.get().getName() + " is NOT ok!", true);
    DriverStation.reportError("Can retry limit exceeded for \"" + action + "\"", false);
    return false;
  }

  /**
   * Given a CTRE Status Code will retry a given amount of times and return true if the code ever
   * returns ok, else will crash the code with an error.
   *
   * @param statusCodeSup The status code.
   * @param retryLimit The retry limit.
   * @return Whether the status code is ok.
   */
  public boolean retryStatusCodeFatal(Supplier<StatusCode> statusCodeSup, String action, int retryLimit) {
    for (int i = 0; i < retryLimit; i++) {
      if (statusCodeSup.get().isOK()) return true;
    }
    throw new CANRetryError(action);
  }

  public enum CANNetwork {
    DRIVE("Drive"),
    SUPERSTRUCTURE("Super"),
    RIO("rio");

    public final String name;
    public final CANBus bus;

    CANNetwork(String name) {
      this.name = name;
      this.bus = new CANBus(name);
    }

    public static CANNetwork fromName(String name) {
      for (CANNetwork bus : values()) {
        if (bus.name.equals(name)) {
          return bus;
        }
      }
      return RIO;
    }
  }

  // public class CharacterizationLogger {
  //   private static final String table = "/Characterization/";

  //   private Optional<MechanismId> activeMechanism = Optional.empty();

  //   final DeviceEntry[] devices;
  //   private final MechanismId mechanism;
  //   private final TimeSensitiveLogger[] loggers;
  //   private final Receiver<TimestampedDouble>[] receivers;
  //   private final StatusSignalQueuer queuer;

  //   public CharacterizationLogger(MechanismId mechanism, DeviceEntry... devices) {
  //     this.mechanism = mechanism;
  //     this.devices = devices;
  //     int signalCount = 0;
  //     for (DeviceEntry device : devices) {
  //       signalCount += device.signals.length;
  //     }
  //     BaseStatusSignal[] signals = new BaseStatusSignal[signalCount];
  //     loggers = new TimeSensitiveLogger[signalCount];
  //     int signalIndex = 0;
  //     for (DeviceEntry device : devices) {
  //       for (int i = 0; i < device.signals.length; i++) {
  //         loggers[signalIndex + i] =
  //             new TimeSensitiveLogger(
  //                 table + mechanism + "/" + device.name + "/" + device.signals[i].getName());
  //       }
  //       System.arraycopy(device.signals, 0, signals, signalIndex, device.signals.length);
  //       signalIndex += device.signals.length;
  //     }
  //     queuer = new StatusSignalQueuer(ConstValues.kCharacterization.FREQUENCY, signals);
  //     receivers = queuer.receivers();
  //   }

  //   public void log() {
  //     if (activeMechanism.isEmpty()) return;
  //     queuer.setRunning(activeMechanism.get() == mechanism);
  //     if (!queuer.isRunning()) return;
  //     for (int i = 0; i < loggers.length; i++) {
  //       loggers[i].log(receivers[i].recvAll());
  //     }
  //   }
  // }

  public static double averageLatency(BaseStatusSignal... signals) {
    double sum = 0;
    for (var signal : signals) {
      sum += signal.getAllTimestamps().getDeviceTimestamp().getLatency();
    }
    return sum / signals.length;
  }

  private record DeviceEntry(
      String name,
      ParentDevice device,
      BaseStatusSignal connectedSignal,
      BaseStatusSignal resetSignal,
      BaseStatusSignal[] signals) {

    public DeviceEntry(String name, ParentDevice device, BaseStatusSignal... signals) {
      this(name, device, getHeartBeatSignal(device), getResetSignal(device), signals);
    }

    public boolean hasReset() {
      return resetSignal.hasUpdated();
    }

    public boolean isConnected() {
      return connectedSignal.getAllTimestamps().getSystemTimestamp().getLatency() < 0.33;
    }
  }

  private class NetworkEntry {
    private final ArrayList<DeviceEntry> devices;
    private BaseStatusSignal[] allSignals;

    public NetworkEntry() {
      this.devices = new ArrayList<>();
      this.allSignals = new BaseStatusSignal[0];
    }

    public DeviceEntry addDevice(String name, ParentDevice device, BaseStatusSignal... signals) {
      final var de = new DeviceEntry(name, device, signals);
      devices.add(de);
      BaseStatusSignal.setUpdateFrequencyForAll(50.0, de.connectedSignal, de.resetSignal);
      final BaseStatusSignal[] newSignals =
          new BaseStatusSignal[allSignals.length + signals.length + 2];
      System.arraycopy(allSignals, 0, newSignals, 0, allSignals.length);
      System.arraycopy(signals, 0, newSignals, allSignals.length, signals.length);
      newSignals[newSignals.length - 2] = de.connectedSignal;
      newSignals[newSignals.length - 1] = de.resetSignal;
      allSignals = newSignals;
      return de;
    }

    @SuppressWarnings("unused")
    public DeviceEntry getDevice(ParentDevice device) {
      for (DeviceEntry de : devices) {
        if (de.device == device) {
          return de;
        }
      }
      return null;
    }
  }

  private final EnumMap<CANNetwork, NetworkEntry> networks =
      new EnumMap<>(CANNetwork.class) {
        {
          for (CANNetwork bus : CANNetwork.values()) {
            put(bus, new NetworkEntry());
          }
        }
      };
  // private final EnumMap<MechanismId, CharacterizationLogger> characterizations =
  //     new EnumMap<>(MechanismId.class);
  private final List<Runnable> deviceLoggers = new ArrayList<>();

  private NetworkEntry getNetworkEntry(String network) {
    return networks.get(CANNetwork.fromName(network));
  }

  /**
   * Bring up a device and log its status signals.
   * This will set the update frequency for all signals to 100Hz and optimize bus utilization.
   * It will also apply the given configuration to the device.
   * <p>
   * The signals that are implicitly logged are:
   * <ul>
   * <li>Position</li>
   * <li>Velocity</li>
   * <li>Acceleration</li>
   * <li>Motor Voltage</li>
   * <li>Torque Current</li>
   * <li>Supply Current</li>
   * <li>Device Temperature</li>
   * <li>Control Mode</li>
   * </ul>
   * <p>
   * @param container the container to log to
   * @param name the name of the device
   * @param device the device to bring up
   * @param config the configuration to apply to the device
   * @param extraSignals additional signals to log
   */
  public void bringUp(
      Logged container,
      String name,
      TalonFX device,
      TalonFXConfiguration config,
      BaseStatusSignal... extraSignals) {
    BaseStatusSignal[] genericSignals = {
      device.getPosition(false),
      device.getVelocity(false),
      device.getAcceleration(false),
      device.getMotorVoltage(false),
      device.getTorqueCurrent(false),
      device.getSupplyCurrent(false),
      device.getDeviceTemp(false),
      device.getControlMode(false)
    };
    Set<BaseStatusSignal> allSignals = new HashSet<>(Arrays.asList(genericSignals));
    allSignals.addAll(Arrays.asList(extraSignals));
    BaseStatusSignal[] allSignalsArray = allSignals.toArray(new BaseStatusSignal[0]);
    retryStatusCodeFatal(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(100.0, allSignalsArray),
        "Update Status Signal Frequency for " + name,
        5
    );
    retryStatusCodeFatal(
      () -> device.optimizeBusUtilization(4.0, 1.0),
      "Optimize Bus Utilization for " + name,
      5
    );
    retryStatusCodeFatal(
      () -> device.getConfigurator().apply(config, 1.0),
      "Apply Config for " + name,
      5
    );
    final var de = getNetworkEntry(device.getNetwork()).addDevice(name, device, allSignalsArray);
    deviceLoggers.add(
        () -> {
          @SuppressWarnings("unchecked")
          var summary =
              new TalonFXSummary(
                  genericSignals[0].getValueAsDouble(),
                  genericSignals[1].getValueAsDouble(),
                  genericSignals[2].getValueAsDouble(),
                  genericSignals[3].getValueAsDouble(),
                  genericSignals[4].getValueAsDouble(),
                  genericSignals[5].getValueAsDouble(),
                  genericSignals[6].getValueAsDouble(),
                  ((StatusSignal<ControlModeValue>) genericSignals[7]).getValue(),
                  de.isConnected(),
                  de.hasReset(),
                  averageLatency(genericSignals));
          container.log(name, CtreStructs.TalonFXSummary.struct, summary);
        });
    Monologue.log(
        "/Devices/" + device.getNetwork() + "/" + name + "/config",
        CtreStructs.TALON_FX_CONFIG_STRUCT,
        config);
  }

  public void bringUp(
      Logged container, String name, CANcoder device, CANcoderConfiguration config) {
    BaseStatusSignal[] genericSignals = {
      device.getAbsolutePosition(false), device.getPosition(false), device.getMagnetHealth(false)
    };
    retryStatusCodeFatal(
      () -> BaseStatusSignal.setUpdateFrequencyForAll(100.0, genericSignals),
      "Update Status Signal Frequency for " + name,
      5
    );
    retryStatusCodeFatal(
      () -> device.optimizeBusUtilization(4.0, 1.0),
      "Optimize Bus Utilization for " + name,
      5
    );
    retryStatusCodeFatal(
      () -> device.getConfigurator().apply(config, 1.0),
      "Apply Config for " + name,
      5
    );
    final var de = getNetworkEntry(device.getNetwork()).addDevice(name, device, genericSignals);
    deviceLoggers.add(
        () -> {
          @SuppressWarnings("unchecked")
          var summary =
              new CtreStructs.CANCoderSummary(
                  genericSignals[0].getValueAsDouble(),
                  genericSignals[1].getValueAsDouble(),
                  ((StatusSignal<MagnetHealthValue>) genericSignals[2]).getValue(),
                  de.isConnected(),
                  de.hasReset(),
                  averageLatency(genericSignals));
          container.log(name, CtreStructs.CANCoderSummary.struct, summary);
        });
    Monologue.log(
        "/Devices/" + device.getNetwork() + "/" + name + "/config",
        CtreStructs.CAN_CODER_CONFIG_STRUCT,
        config);
  }

  public void bringUp(Logged container, String name, Pigeon2 device, Pigeon2Configuration config) {
    String summaryPath = name + "/summary";
    BaseStatusSignal[] genericSignals = {
      device.getAccelerationX(false),
      device.getAccelerationY(false),
      device.getRoll(false),
      device.getPitch(false),
      device.getYaw(false)
    };
    retryStatusCodeFatal(
      () -> BaseStatusSignal.setUpdateFrequencyForAll(100.0, genericSignals),
      "Update Status Signal Frequency for " + name,
      5
    );
    retryStatusCodeFatal(
      () -> device.optimizeBusUtilization(4.0, 1.0),
      "Optimize Bus Utilization for " + name,
      5
    );
    retryStatusCodeFatal(
      () -> device.getConfigurator().apply(config, 1.0),
      "Apply Config for " + name,
      5
    );
    final var de = getNetworkEntry(device.getNetwork()).addDevice(name, device, genericSignals);
    deviceLoggers.add(
        () -> {
          var summary =
              new CtreStructs.Pigeon2Summary(
                  genericSignals[0].getValueAsDouble(),
                  genericSignals[1].getValueAsDouble(),
                  genericSignals[2].getValueAsDouble(),
                  genericSignals[3].getValueAsDouble(),
                  genericSignals[4].getValueAsDouble(),
                  de.isConnected(),
                  de.hasReset(),
                  averageLatency(genericSignals));
          container.log(summaryPath, CtreStructs.Pigeon2Summary.struct, summary);
        });
    container.log(name + "/config", CtreStructs.PIGEON_2_CONFIG_STRUCT, config);
  }

  public void bringUp(Logged container, String name, CANrange device, CANrangeConfiguration config) {
    String summaryPath = name + "/summary";
    BaseStatusSignal[] genericSignals = {
      device.getAmbientSignal(false),
      device.getDistance(false),
      device.getIsDetected(false)
    };
    retryStatusCodeFatal(
      () -> BaseStatusSignal.setUpdateFrequencyForAll(100.0, genericSignals),
      "Update Status Signal Frequency for " + name,
      5
    );
    retryStatusCodeFatal(
      () -> device.optimizeBusUtilization(4.0, 1.0),
      "Optimize Bus Utilization for " + name,
      5
    );
    retryStatusCodeFatal(
      () -> device.getConfigurator().apply(config, 1.0),
      "Apply Config for " + name,
      5
    );
    final var de = getNetworkEntry(device.getNetwork()).addDevice(name, device, genericSignals);
    deviceLoggers.add(
        () -> {
          var summary =
              new CtreStructs.CANRangeSummary(
                  genericSignals[0].getValueAsDouble(),
                  genericSignals[1].getValueAsDouble(),
                  genericSignals[2].getValueAsDouble(),
                  de.isConnected(),
                  de.hasReset(),
                  averageLatency(genericSignals));
          container.log(summaryPath, CtreStructs.CANRangeSummary.struct, summary);
        });
    container.log(name + "/config", CtreStructs.CAN_RANGE_CONFIG_STRUCT, config);
  }

  // public void assignDevicesToMechanismCharacterization(MechanismId id, ParentDevice... devices) {
  //   if (characterizations.containsKey(id)) {
  //     var charLogger = characterizations.remove(id);
  //     ParentDevice[] newDevices = new ParentDevice[devices.length + charLogger.devices.length];
  //     System.arraycopy(charLogger.devices, 0, newDevices, 0, charLogger.devices.length);
  //     System.arraycopy(devices, 0, newDevices, charLogger.devices.length, devices.length);
  //     devices = newDevices;
  //   }
  //   final NetworkEntry ne = getNetworkEntry(devices[0].getNetwork());
  //   DeviceEntry[] deviceEntries = new DeviceEntry[devices.length];
  //   for (int i = 0; i < devices.length; i++) {
  //     deviceEntries[i] = ne.getDevice(devices[i]);
  //   }
  //   final CharacterizationLogger logger = new CharacterizationLogger(id, deviceEntries);
  //   characterizations.put(id, logger);
  // }

  public boolean ifAnyAreDisconnected(ParentDevice... devices) {
    for (ParentDevice device : devices) {
      if (!getNetworkEntry(device.getNetwork()).getDevice(device).isConnected()) {
        return true;
      }
    }
    return false;
  }

  public boolean ifAnyAreDisconnected(String network) {
    for (DeviceEntry device : getNetworkEntry(network).devices) {
      if (!device.isConnected()) {
        return true;
      }
    }
    return false;
  }

  public boolean ifAnyAreDisconnected() {
    for (NetworkEntry network : networks.values()) {
      for (DeviceEntry device : network.devices) {
        if (!device.isConnected()) {
          return true;
        }
      }
    }
    return false;
  }

  /** Refreshes all signals in the CANSignalManager, should be called once per cycle */
  public void update() {
    if (Robot.isSimulation()) {
      return;
    }
    for (Entry<CANNetwork, NetworkEntry> entry : networks.entrySet()) {
      if (entry.getValue().allSignals.length != 0) {
        continue;
      }
      Tracer.startTrace(entry.getKey().name());
      BaseStatusSignal.refreshAll(entry.getValue().allSignals);
      Tracer.endTrace();
    }
    for (Runnable logger : deviceLoggers) {
      logger.run();
    }
  }
}
