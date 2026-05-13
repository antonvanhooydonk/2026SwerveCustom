// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.drive;

// import static edu.wpi.first.units.Units.Degrees;

// import com.studica.frc.Navx;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.util.sendable.Sendable;
// import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import frc.robot.Constants.CANConstants;

// /**
//  * Wrapper class for NavX3-CAN gyro that is used by the robot's swerve drive.
//  */
// public class Gyro implements Sendable {    
//   // Hardware components
//   private final Navx gyro = new Navx(CANConstants.kGyroID);

//   // Time threshold to consider gyro data stale if angle hasn't changed
//   private static final double kGyroStaleTimeSeconds = 0.5;

//   // Track connection status based on whether gyro angle is updating
//   private boolean connected = false;
//   private double lastAngle = 0.0;
//   private double lastAngleChangeTime = 0.0;
  
//   /**
//    * Creates a new Gyro object.
//    */
//   public Gyro() {
//     try {
//       // Inform the driver that the gyro is initializing and needs to be still
//       System.out.println("Gyro warming up, please keep the robot still for 5 seconds...");

//       // Disable certain gyro messages to optimize CAN bus saturation
//       gyro.enableOptionalMessages(
//         true,
//         true,
//         false,
//         false,
//         false,
//         true,
//         false,
//         false,
//         false,
//         true
//       );

//       // Instead of isConnected(), we perform a "warm-up" wait.
//       // The NavX3-CAN needs ~5 seconds of stillness to establish its 
//       // initial bias for the lowest possible drift (~0.0582°/min).
//       Timer.delay(5.0); 
//       System.out.println("Gyro warm up complete.");

//       // After the delay, zero the yaw to set your starting heading
//       gyro.resetYaw();        
//     } 
//     catch (Exception ex) {
//       DriverStation.reportError("NavX3-CAN failed to initialize: " + ex.getMessage(), true);
//     }

//     // Initialize dashboard values
//     SmartDashboard.putData("Drive/Gyro", this);
//   }

//   /**
//    * Periodic method to be called by drive subsystem's periodic
//    */
//   public void periodic() {
//     checkConnection();
//   }

//   /**
//    * Updates the internal connection status based on whether the gyro angle
//    * has changed within the last 0.5 seconds while the robot is moving.
//    */
//   private void checkConnection() {
//     try {
//       double currentAngle = getAngle().getDegrees();
//       if (currentAngle != lastAngle) {
//         lastAngle = currentAngle;
//         lastAngleChangeTime = Timer.getFPGATimestamp();
//       }
//       connected = (Timer.getFPGATimestamp() - lastAngleChangeTime) < kGyroStaleTimeSeconds;
//     } 
//     catch (Exception ex) {
//       connected = false;
//     }
//   }

//   /**
//    * Gets whether the gyro appears to be connected and updating.
//    * Returns false if the gyro angle has not changed for 0.5 
//    * seconds while the robot is moving.
//    */
//   public boolean isConnected() {
//     return connected;
//   }

//   /**
//    * Gets the current gyro angle. This may not match the robot's heading
//    * due to initial offset, or drift over time. Generally, this is only 
//    * used as input into the swerve drive PoseEstimator, and then our 
//    * robot can be driven based on the PoseEstimator's heading.
//    * @return The current gyro angle as a Rotation2d, CCW positive
//    */
//   public Rotation2d getAngle() {
//     return gyro.getRotation2d();
//   }

//   /**
//    * Gets the current roll of the robot
//    * @return Current roll in degrees
//    */
//   public double getRoll() {
//     return gyro.getRoll().in(Degrees);
//   }
  
//   /**
//    * Gets the current pitch of the robot (for auto-balancing)
//    * @return Current pitch in degrees
//    */
//   public double getPitch() {
//     return gyro.getPitch().in(Degrees);
//   }
  
//   /**
//    * Gets the current yaw of the robot
//    * @return Current yaw in degrees -180 to 180
//    */
//   public double getYaw() {
//     return gyro.getYaw().in(Degrees);
//   }

//   /**
//    * Resets the gyro to zero and clears internal offset.
//    */
//   public void reset() {
//     gyro.resetYaw();
//   }

//   /**
//    * Initialize the data sent to SmartDashboard
//    */
//   @Override
//   public void initSendable(SendableBuilder builder) {
//     builder.setSmartDashboardType("Gyro");
//     builder.addDoubleProperty("Angle (degs)", () -> getAngle().getDegrees(), null);
//     builder.addDoubleProperty("Roll (degs)", this::getRoll, null);
//     builder.addDoubleProperty("Pitch (degs)", this::getPitch, null);
//     builder.addDoubleProperty("Yaw (degs)", this::getYaw, null);
//   }
// }
