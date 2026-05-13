// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.Utils;

/**
 * Wrapper class for Thrifty Bot absolute encoder attached to swerve steer motor.
 * Provides both raw AnalogInput access and processed AnalogEncoder functionality.
 */
public class AbsoluteEncoder implements Sendable {    
  // Hardware components
  private final AnalogInput analogInput;
  private final AnalogEncoder analogEncoder;
  
  // Configuration
  private final String name;
  private double offsetRadians;
  private boolean inverted = false;
  
  // Voltage thresholds for diagnostics
  private final double VOLTAGE_NOISE_THRESHOLD = 0.1;
  
  // Previous reading for noise detection
  private double previousVoltage = 0.0;
  private int noiseCount = 0;
  
  /**
   * Creates a new ThriftyBotEncoder
   * @param analogPort The analog input port (0-3 on roboRIO)
   * @param offsetRadians Offset in radians to zero the encoder
   * @param inverted True if the encoder direction should be inverted
   * @param name Name for debugging and dashboard
   */
  public AbsoluteEncoder(int analogPort, double offsetRadians, boolean inverted, String name) {
    this.name = name;
    this.offsetRadians = offsetRadians;
    this.inverted = inverted;
    
    // Initialize both analog input and encoder
    this.analogInput = new AnalogInput(analogPort);
    this.analogEncoder = new AnalogEncoder(analogInput);

    // Initialize dashboard values
    SmartDashboard.putData("Absolute Encoder/" + name, this);
  }
  
  /**
   * Periodic method to be called regularly for diagnostics
   */
  public void periodic() {
    // Read current voltage
    double currentVoltage = getRawVoltage();
    
    // Check for noise
    if (Math.abs(currentVoltage - previousVoltage) > VOLTAGE_NOISE_THRESHOLD) {
      // Detected a significant voltage change, likely noise or disconnection
      noiseCount++;
    } else {
      // Decay noise count over time
      noiseCount = Math.max(0, noiseCount - 1);
    }
    
    // Update previous voltage for next cycle
    previousVoltage = currentVoltage;
  }

  /**
   * Gets the current angle in radians using AnalogEncoder.
   * The current offset and inversion are applied if configured.
   * @return Angle in radians (-π to π)
   */
  public double getAngleRadians() {
    // Get position from AnalogEncoder (0.0 to 1.0 rotations)
    double position = analogEncoder.get();
    
    // Convert position to radians (0 to 2π)
    double angle = position * 2.0 * Math.PI;
    
    // Apply offset
    angle -= offsetRadians;
    
    // Apply inversion if needed
    if (inverted) {
      angle = -angle;
    }

    // Return the angle normalized to (-π, π)
    return Utils.normalizeAngle(angle);
  }
  
  /**
   * Gets the current angle as a Rotation2d.
   * The current offset and inversion are applied if configured.
   * @return Rotation2d object
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromRadians(getAngleRadians());
  }
  
  /**
   * Gets the current angle in degrees.
   * The current offset and inversion are applied if configured.
   * @return Angle in degrees (0 to 360) 
   */
  public double getAngleDegrees() {
    return Units.radiansToDegrees(getAngleRadians());
  }
  
  /**
   * Gets the raw voltage from the analog input
   * @return Voltage (0-5V)
   */
  public double getRawVoltage() {
    return analogInput.getVoltage();
  }
  
  /**
   * Gets the current raw angle in radians from the analog input.
   * Use this value directly for setting the module offsets in DriveConstants.
   * @return Raw angle in radians (0 to 2π) 
   */
  public double getRawAngleRadians() {
    double voltage = getRawVoltage();
    double supplyVoltage = RobotController.getVoltage5V();
    double angle = (voltage / supplyVoltage) * 2.0 * Math.PI;    
    if (inverted) {
      angle = -angle;
    }
    return Utils.normalizeAngle(angle);
  }
  
  /**
   * Gets the current raw angle (no offset) in degrees from the analog input.
   * Use this value with Units.degreesToRadians(value) for setting the
   * module offsets in DriveConstants.
   * @return Raw angle in degrees
   */
  public double getRawAngleDegrees() {
    return Units.radiansToDegrees(getRawAngleRadians());
  }
  
  /**
   * Gets the absolute position (0 to 1.0 rotations).
   * This method does not apply the internal offset.
   * @return Position as fraction of full rotation
   */
  public double getAbsolutePosition() {
    return analogEncoder.get();
  }
  
  /**
   * Sets whether the encoder direction should be inverted
   * @param inverted True to invert the direction
   */
  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }
  
  /**
   * Gets whether the encoder is inverted
   * @return True if inverted
   */
  public boolean isInverted() {
    return inverted;
  }
  
  /**
   * Gets the current offset
   * @return Current offset in radians
   */
  public double getOffset() {
    return offsetRadians;
  }
  
  /**
   * Checks if the encoder readings are valid
   * @return True if readings appear valid
   */
  public boolean isValid() {
    // Consider valid if noise count is below threshold
    return noiseCount < 10;
  }

  /**
   * Gets the current noise count for diagnostics
   * @return
   */
  public int getNoiseCount() {
    return noiseCount;
  }

  /**
   * Gets the analog input port number
   * @return Port number
   */
  public int getPort() {
    return analogInput.getChannel();
  }

  /**
   * Gets the encoder name
   * @return Name string
   */
  public String getName() {
    return name;
  }

  /**
   * Closes the encoder and frees resources
   */
  public void close() {
    analogInput.close();
    analogEncoder.close();
  }

  /**
   * Initialize the data sent to SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Absolute Position", () -> Utils.showDouble(getAbsolutePosition()), null);
    builder.addDoubleProperty("Angle (deg)", () -> Utils.showDouble(getAngleDegrees()), null);
    builder.addDoubleProperty("Angle (rad)", () -> Utils.showDouble(getAngleRadians()), null);
    builder.addDoubleProperty("Raw Angle (deg)", () -> Utils.showDouble(getRawAngleDegrees()), null);
    builder.addDoubleProperty("Raw Angle (rad)", () -> Utils.showDouble(getRawAngleRadians()), null);
    builder.addDoubleProperty("Raw Voltage", () -> Utils.showDouble(getRawVoltage()), null);
    builder.addDoubleProperty("Supply Voltage", () -> Utils.showDouble(RobotController.getVoltage5V()), null);
    builder.addBooleanProperty("Valid", this::isValid, null);
    builder.addBooleanProperty("Inverted", this::isInverted, null);
    builder.addIntegerProperty("Noise Count", this::getNoiseCount, null);
  }
}
