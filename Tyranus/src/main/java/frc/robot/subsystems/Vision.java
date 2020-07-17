// package frc.robot.subsystems;

// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.SwerveDrive;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.command.InstantCommand;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.subsystems.*;


// public class Vision extends SubsystemBase {
//     public static SwerveDrive swerveDrive = new SwerveDrive();
//     private final Conveyor zoom = new Conveyor();
//     public final static Shooter shooter = new Shooter();

//     NetworkTable table;

//     NetworkTableEntry targetX; // horizontal angle
//     NetworkTableEntry targetY; // vertical angle

//     double rotationError; // Rotation error value
//     double distanceError; // Distance error value

//     // Deadzone is necessary becasue the robot can only get so accurate and cannot be perfectly head on target
//     double angleTolerance = 5; // Deadzone for the angle control loop
//     double distanceTolerance = 5; // Deadzone for the distance control loop

//     double rotationAjust; // rotational signal for the drivetrain (?)
//     double distanceAjust; // distance signal for the drivetrain (?)

//     // these may be specific to the drivetrain used in the docs...?
//     double KpRot=-0.1;
//     double KpDist=-0.1;

//     double constantForce = 0.05;

//     double random; // really confused at this point

//     double xDistance = 0; // to be used in later function triggy()
//     double yDistance = 0; // ditto ^
//     Rotation2d currentRotationSwerve = new Rotation2d();
//     SwerveControllerCommand visionCommand;

//     // to be called in RobotContainer
//     public Vision() {
        
//         // initializes "table" as the Network Tables database named "chameleon-vision"
//         table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("CameraName");

//         // points targets to database values
//         targetX = table.getEntry("yaw");
//         targetY = table.getEntry("pitch");

//         SmartDashboard.putNumber("idk", 4);
//     }

//     public Command runVision() {
//         rotationAjust = 0; // not sure what this does
//         distanceAjust = 0; // maybe: will be used to tell the robot how much to move

//         // control loop for rotation
//         if (rotationError > angleTolerance) {
//             rotationAjust = KpRot * rotationError + constantForce;
//         } else if (rotationError < angleTolerance) {
//             rotationAjust = KpRot * rotationError - constantForce;
//         }

//         // control loop for distance
//         if (distanceError > distanceTolerance) {
//             distanceAjust = KpDist * distanceError + constantForce;
//         } else if (distanceError < distanceTolerance) {
//             distanceAjust = KpDist * distanceError - constantForce;
//         }

//         // *** ATTEMPT #1 *** - simply using drive()
//         // line up swerve/drivetrain with target
//         // somehow convert rotation & distance into x & y values - trigonometry

//         // probably have to reset gyro --- wait no i think that'd be bad

//         // this doesn't take the robot's current position into account i think
//         // triggy(rotationAjust, distanceAjust);
//         respectfulTriggy(rotationAjust, distanceAjust);
      
        
//         // run it all and shoot!
// // this didn't work
//         // visionCommand = new RunCommand(() -> RobotContainer.swerveDrive.drive(xDistance,
//         //                                                                       yDistance,
//         //                                                                       rotationAjust, // not really sure what to put for rot
//         //                                                                       true), 
//         //                                swerveDrive);
//         // call the command -- don't know how to do that (where do commands get ultimately called?)

// //  return visionCommand;

//         // *** ATTEMPT #2 *** - using trajectories
//         // nevermind it seems a lot more complicated
//     }

//     public void filler() {
//         double random = 0;
//     }
    
//     // trigonometry to figure our x and y coordinates
//     public void triggy(double rotation, double distance) {
//         // x-part of triangle = cos(ANGLE) * HYPOTENUSE
//         xDistance = Math.cos(rotation) * distance;

//         // y-part of triangle = sin(ANGLE) * HYPOTENUSE
//         yDistance = Math.sin(rotation) * distance;
//     }

//     // as in with respect to current values
//     public void respectfulTriggy(double rotation, double distance) {
//         // double firstXDistance = Math.cos(rotation) * distance;
//         if (currentRotationSwerve.getCos() > rotation) {
//             xDistance = (distance * (currentRotationSwerve.getCos() - rotation));
//         } else {
//             xDistance = (distance * (rotation - currentRotationSwerve.getCos()));
//         }

//         if (currentRotationSwerve.getSin() > rotation) {
//             yDistance = (distance * (currentRotationSwerve.getSin() - rotation));
//         } else {
//             yDistance = (distance * (rotation - currentRotationSwerve.getSin()));
//         }
//     }

// } 