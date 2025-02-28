package igknighters.subsystems.vision.camera;

import edu.wpi.first.math.geometry.Rotation2d;
import igknighters.SimCtx;
import java.util.function.Function;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

@SuppressWarnings("unused")
public class CameraSimPhoton extends CameraRealPhoton {
  private final SimCtx simCtx;

  public CameraSimPhoton(
      CameraConfig config, SimCtx simCtx, Function<Double, Rotation2d> gyroYawSupplier) {
    super(config, gyroYawSupplier);
    this.simCtx = simCtx;

    final SimCameraProperties props = new SimCameraProperties();
    props.setCalibError(0.05, 0.002);
    props.setFPS(43.0);
    props.setCalibration(1280, 800, Rotation2d.fromDegrees(79.1));
    props.setAvgLatencyMs(20.0);
    props.setLatencyStdDevMs(2.0);
    final PhotonCameraSim sim = new PhotonCameraSim(camera, props, 0.12, 6.5);
    sim.enableDrawWireframe(true);
    simCtx.aprilTagSim().addCamera(sim, config.cameraTransform());
  }
}
