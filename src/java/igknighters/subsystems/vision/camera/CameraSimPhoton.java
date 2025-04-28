package igknighters.subsystems.vision.camera;

import igknighters.SimCtx;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

@SuppressWarnings("unused")
public class CameraSimPhoton extends CameraRealPhoton {
  private final SimCtx simCtx;

  public CameraSimPhoton(CameraConfig config, SimCtx simCtx) {
    super(config);
    this.simCtx = simCtx;

    final SimCameraProperties props = new SimCameraProperties();
    props.setCalibError(0.5, 0.02);
    props.setFPS(43.0);
    props.setCalibration(
        1280, 800, config.intrinsics().cameraMatrix(), config.intrinsics().distortionMatrix());
    props.setAvgLatencyMs(35.0);
    props.setLatencyStdDevMs(6.0);
    final PhotonCameraSim sim = new PhotonCameraSim(camera, props, 0.12, 6.5);
    sim.enableDrawWireframe(true);
    simCtx.aprilTagSim().addCamera(sim, config.cameraTransform());
  }
}
