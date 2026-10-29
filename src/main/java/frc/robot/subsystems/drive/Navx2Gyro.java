// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.Utils;

/**
 * Wrapper class for NavX3-CAN gyro that is used by the robot's swerve drive.
 */
public class Navx2Gyro implements Sendable {    
  // Hardware components
  private final AHRS gyro;

  private static final double kCalibrationTimeoutSeconds = 20.0;

  private final double bootTimeSeconds = Timer.getFPGATimestamp();
  private boolean calibrationReported = false;
  private boolean calibrationTimedOut = false;
  
  private final Alert disconnectedAlert = new Alert("NavX gyro disconnected! Heading is unreliable.", AlertType.kError);
  private final Alert calibrationAlert = new Alert("NavX calibration timed out at boot. Heading may be bad.", AlertType.kError);
  private final Alert magAlert = new Alert("NavX magnetometer not calibrated.", AlertType.kWarning);
  
  // Simulated heading (CCW positive). Only used when running in simulation, since the
  // real AHRS has no physical robot to measure there.
  private Rotation2d simAngle = Rotation2d.kZero;

  // Debounce so a single dropped SPI frame doesn't flash an error
  private final Debouncer disconnectDebouncer = new Debouncer(0.25, DebounceType.kRising);

  /**
   * Creates a new Navx2Gyro object.
   */
  public Navx2Gyro() {
    // Create the gyro
    gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k100Hz);

    // Set the gyro angle adjustment to match the physical mounting of the gyro on the robot
    gyro.setAngleAdjustment(DriveConstants.kGyroAngleOffsetDegrees);

    // Initialize dashboard values
    SmartDashboard.putData("Drive/Gyro", this);
  }

  /**
   * Periodic method to be called by drive subsystem's periodic
   */
  public void periodic() {
    // The simulated gyro is always connected and calibrated
    if (RobotBase.isSimulation()) {
      return;
    }

    boolean connected = gyro.isConnected();
    boolean calibrating = gyro.isCalibrating();

    // Report calibration result once, whenever it actually finishes
    if (connected && !calibrating && !calibrationReported) {
      calibrationReported = true;
      calibrationTimedOut = false;
      if (gyro.isMagnetometerCalibrated()) {
        Utils.logInfo("Gyro calibration completed successfully.");
      } else {
        magAlert.set(true);
        Utils.logError("Gyro calibration complete. Magnetometer not calibrated!");
      }
    }

    // Flag (once) if calibration is taking too long, but keep waiting
    if (
      calibrating && 
      !calibrationTimedOut && 
      Timer.getFPGATimestamp() - bootTimeSeconds > kCalibrationTimeoutSeconds
    ) {
      calibrationTimedOut = true;
      Utils.logError("Gyro calibration timed out!");
    }

    disconnectedAlert.set(disconnectDebouncer.calculate(!connected));
    calibrationAlert.set(calibrationTimedOut && calibrating);
  }

  /** 
   * True while the NavX is still calibrating. 
   */
  public boolean isCalibrating() {
    if (RobotBase.isSimulation()) {
      return false;
    }
    return gyro.isCalibrating();
  }

  /** 
   * True when the gyro is connected and finished calibrating, so its angle can be trusted.
   */
  public boolean isReady() {
    if (RobotBase.isSimulation()) {
      return true;
    }
    return gyro.isConnected() && !gyro.isCalibrating();
  }

  /**
   * Gets the current gyro angle. This may not match the robot's heading
   * due to initial offset, or drift over time. Generally, this is only 
   * used as input into the swerve drive PoseEstimator, and then our 
   * robot can be driven based on the PoseEstimator's heading.
   * @return The current gyro angle as a Rotation2d, CCW positive
   */
  public Rotation2d getAngle() {
    if (RobotBase.isSimulation()) {
      return simAngle;
    }
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
   * Gets the current yaw of the robot, CCW positive like getAngle().
   * Unlike getAngle(), this does not include the angle adjustment 
   * or accumulate past +/-180 degrees.
   * @return Current yaw in degrees -180 to 180, CCW positive
   */
  public double getYaw() {
    if (RobotBase.isSimulation()) {
      return simAngle.getDegrees();
    }
    return -gyro.getYaw();
  }

  /**
   * Resets the gyro to zero and clears internal offset.
   */
  public void reset() {
    simAngle = Rotation2d.kZero;
    gyro.reset();
  }

  /**
   * SIMULATION ONLY: Integrates the robot's true angular velocity into the simulated heading.
   * Call once per loop from the drive subsystem's simulationPeriodic().
   * @param omegaRadPerSec Robot angular velocity, CCW positive
   * @param dtSeconds Time step in seconds
   */
  public void simulate(double omegaRadPerSec, double dtSeconds) {
    simAngle = simAngle.plus(Rotation2d.fromRadians(omegaRadPerSec * dtSeconds));
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
