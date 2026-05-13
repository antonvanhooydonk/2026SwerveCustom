// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.Utils;

/**
 * Wrapper class for NavX3-CAN gyro that is used by the robot's swerve drive.
 */
public class Navx2Gyro implements Sendable {    
  // Hardware components
  private final AHRS gyro;
  
  /**
   * Creates a new Navx2Gyro object.
   */
  public Navx2Gyro() {
    // Create the gyro
    gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k100Hz);

    // Wait for gyro to calibrate itself on startup
    waitForGyroCalibration();

    // Set the gyro angle adjustment to match the physical mounting of the gyro on the robot
    gyro.setAngleAdjustment(SwerveConstants.kGyroAngleOffsetDegrees);

    // Initialize dashboard values
    SmartDashboard.putData("Drive/Gyro", this);
  }

  /**
   * Periodic method to be called by drive subsystem's periodic
   */
  public void periodic() {}

  /**
   * Waits for gyro calibration to complete (with timeout)
   */
  private void waitForGyroCalibration() {
    boolean timeout = false;
    int waitCount = 0;

    while (gyro.isCalibrating() && timeout == false) {
      // Wait for 1 second
      Timer.delay(1.0);
      waitCount++;
      
      // Print message every second
      Utils.logInfo("Calibrating gyro... (" + waitCount + "s). Do not move the robot!");
      
      // 20 seconds timeout
      if (waitCount > 20) { 
        timeout = true;
      }
    }

    // Print calibration complete message
    if (timeout) {
      Utils.logError("Gyro calibration timed out!");
    }
    else if (!gyro.isMagnetometerCalibrated()) {
      Utils.logError("Gyro calibration complete. Magnetometer not calibrated!");
    }
    else {
      Utils.logInfo("Gyro calibration completed successfully.");
    }
  }

  /**
   * Gets whether the gyro appears to be connected and updating.
   * Returns false if the gyro angle has not changed for 0.5 
   * seconds while the robot is moving.
   */
  public boolean isConnected() {
    return gyro.isConnected();
  }

  /**
   * Gets the current gyro angle. This may not match the robot's heading
   * due to initial offset, or drift over time. Generally, this is only 
   * used as input into the swerve drive PoseEstimator, and then our 
   * robot can be driven based on the PoseEstimator's heading.
   * @return The current gyro angle as a Rotation2d, CCW positive
   */
  public Rotation2d getAngle() {
    return gyro.getRotation2d();
  }

  /**
   * Gets the current roll of the robot
   * @return Current roll in degrees
   */
  public double getRoll() {
    return gyro.getRoll();
  }
  
  /**
   * Gets the current pitch of the robot (for auto-balancing)
   * @return Current pitch in degrees
   */
  public double getPitch() {
    return gyro.getPitch();
  }
  
  /**
   * Gets the current yaw of the robot
   * @return Current yaw in degrees -180 to 180
   */
  public double getYaw() {
    return gyro.getYaw();
  }

  /**
   * Resets the gyro to zero and clears internal offset.
   */
  public void reset() {
    gyro.reset();
  }

  /**
   * Initialize the data sent to SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Angle (degs)", () -> getAngle().getDegrees(), null);
    builder.addDoubleProperty("Roll (degs)", this::getRoll, null);
    builder.addDoubleProperty("Pitch (degs)", this::getPitch, null);
    builder.addDoubleProperty("Yaw (degs)", this::getYaw, null);
  }
}
