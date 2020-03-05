package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.vision.GripPipeline;

// import org.opencv.core.Rect;
// import org.opencv.imgproc.Imgproc;

// import edu.wpi.cscore.UsbCamera;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.wpilibj.vision.VisionRunner;
// import edu.wpi.first.wpilibj.vision.VisionThread;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  // private static final int IMG_WIDTH = 320;
  // private static final int IMG_HEIGHT = 240;

  // private VisionThread visionThread;
  // private double centerX = 0.0;

  // private final Object imgLock = new Object();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    // camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    // visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
    //     if (!pipeline.convexHullsOutput().isEmpty()) {
    //         Rect r = Imgproc.boundingRect(pipeline.convexHullsOutput().get(0));
    //         synchronized (imgLock) {
    //             centerX = r.x + (r.width / 2);
    //         }
    //     }
    // });
    // visionThread.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // double centerX;
    // synchronized (imgLock) {
    //     centerX = this.centerX;
    // }
    // double center = centerX - (IMG_WIDTH / 2);
    // double yaw = ((center - ((IMG_WIDTH / 2) - 0.5)) * (68.5/IMG_WIDTH));
    // double pitch = ((center - ((IMG_HEIGHT / 2) - 0.5)) * (68.5/IMG_HEIGHT));
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    RobotContainer.swerveDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(-Math.PI / 2.)));
    RobotContainer.theta.reset(-Math.PI / 2.);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    RobotContainer.swerveDrive.resetEncoders();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.swerveDrive.resetEncoders();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
