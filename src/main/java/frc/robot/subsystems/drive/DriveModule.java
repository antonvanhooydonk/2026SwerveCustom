// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Newtons;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.util.Conversions;
import frc.robot.util.SparkMaxFactory;
import frc.robot.util.TalonFXFactory;
import frc.robot.util.Utils;

/**
 * Represents a single swerve drive module with a Kraken x60 drive motor
 * and a Neo steering motor with a ThriftyBot absolute encoder attached.
 */
public class DriveModule implements Sendable {
  // Hardware
  private final TalonFX driveMotor;
  private final SparkMax steerMotor;
  private final AbsoluteEncoder absoluteEncoder;

  // Controllers
  private final SparkClosedLoopController steerPIDController;
  private final RelativeEncoder steerEncoder;
  private final VelocityVoltage driveVelocityRequest;

  // Module configuration
  private final int driveMotorID;
  private final int steerMotorID;
  private final boolean driveMotorInverted;
  private final boolean steerMotorInverted;
  private final int absoluteEncoderID;
  private final double absoluteEncoderOffsetRadians;
  private final boolean absoluteEncoderInverted;
  private final String moduleName;

  // Cached state & position for optimization
  private SwerveModuleState cachedState;
  private SwerveModulePosition cachedPosition;

  // Target state (used for telemetry only)
  private SwerveModuleState targetState;

  // Counter for encoder drift detection
  private int driftLoops = 0;

  // Status signals for drive motor telemetry
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveMotorVoltage;
  private final StatusSignal<Current> driveSupplyCurrent;
  private final StatusSignal<Temperature> driveTemp;

  /**
   * Construct a SwerveModule with the given parameters
   * @param moduleName Name of the module for debugging
   * @param driveMotorID CAN ID of the drive motor
   * @param steerMotorID CAN ID of the steering motor
   * @param driveMotorInverted Indicates that the drive motor is inverted
   * @param steerMotorInverted Indicates that the steer motor is inverted
   * @param absoluteEncoderID Analog ID of the absolute encoder
   * @param absoluteEncoderOffsetRadians Offset angle in radians for the absolute encoder
   * @param absoluteEncoderInverted Indicates that the absolute encoder is inverted
   */
  public DriveModule(
    String moduleName,
    int driveMotorID,
    int steerMotorID,
    boolean driveMotorInverted,
    boolean steerMotorInverted,
    int absoluteEncoderID,
    double absoluteEncoderOffsetRadians,
    boolean absoluteEncoderInverted
  ) {
    // Cache module info
    this.moduleName = moduleName;
    this.driveMotorID = driveMotorID;
    this.steerMotorID = steerMotorID;
    this.driveMotorInverted = driveMotorInverted;
    this.steerMotorInverted = steerMotorInverted;
    this.absoluteEncoderID = absoluteEncoderID;
    this.absoluteEncoderOffsetRadians = absoluteEncoderOffsetRadians;
    this.absoluteEncoderInverted = absoluteEncoderInverted;

    // Initialize the drive motor
    driveMotor = TalonFXFactory.createMotor(this.driveMotorID, getDriveMotorConfig());

    // Initialize the steering motor
    steerMotor = SparkMaxFactory.createMotor(this.steerMotorID, getSteerMotorConfig());

    // Initialize the absolute encoder
    absoluteEncoder = new AbsoluteEncoder(
      this.absoluteEncoderID, 
      this.absoluteEncoderOffsetRadians, 
      this.absoluteEncoderInverted,
      this.moduleName
    );

    // Initialize the drive velocity request
    driveVelocityRequest = new VelocityVoltage(0).withSlot(0);

    // Initialize drive motor status signals
    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveMotorVoltage = driveMotor.getMotorVoltage();
    driveSupplyCurrent = driveMotor.getSupplyCurrent();
    driveTemp = driveMotor.getDeviceTemp();

    // Get the steering motor's onboard PID controller
    steerPIDController = steerMotor.getClosedLoopController();

    // Get the steering motor's relative encoder
    steerEncoder = steerMotor.getEncoder();

    // Reset encoders (do this after getting the encoder)
    resetEncoders();

    // Initialize cached state & position
    targetState = new SwerveModuleState(0.0, Rotation2d.kZero);
    cachedState = new SwerveModuleState(0.0, Rotation2d.kZero);
    cachedPosition = new SwerveModulePosition(0.0, Rotation2d.kZero);

    // Initialize dashboard values
    SmartDashboard.putData("Drive/Modules/" + this.moduleName, this);

    // Output initialization progress
    Utils.logInfo(this.moduleName + " swerve module initialized");
  }

  /**
   * Gets the Kraken x60 drive motor configuration.
   */
  private TalonFXConfiguration getDriveMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Motor outputs
    config.MotorOutput
      .withDutyCycleNeutralDeadband(0.001)  // 0.1% deadband (tight control)
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(driveMotorInverted 
        ? InvertedValue.Clockwise_Positive 
        : InvertedValue.CounterClockwise_Positive
      );
    
    // Current limits (hardcoded here for safety)
    config.CurrentLimits
      .withSupplyCurrentLimitEnable(true)   // Enable supply limits
      .withSupplyCurrentLimit(60)                 // Peak current spike limit in Amps
      .withSupplyCurrentLowerLimit(40)       // Continuous current limit in Amps
      .withSupplyCurrentLowerTime(0.5)        // Time until lower current in seconds
      .withStatorCurrentLimitEnable(true)   // Enable stator limits
      .withStatorCurrentLimit(80);                // Max stator current in Amps (prevents overheating)

    // Voltage compensation
    config.Voltage
      .withPeakForwardVoltage(12)                   // Max voltage when running motor forward
      .withPeakReverseVoltage(-12)                                        // Max voltage when running motor in reverse
      .withSupplyVoltageTimeConstant(0.02);  // Voltage filter time constant in seconds

    // Velocity PID (runs on onboard motor controller, tunable in constants)
    config.Slot0
      .withKP(DriveConstants.kDriveKP)      // Proportional gain
      .withKI(DriveConstants.kDriveKI)      // Integral gain
      .withKD(DriveConstants.kDriveKD)      // Derivative gain
      .withKS(DriveConstants.kDriveKS)      // Static feedforward
      .withKV(DriveConstants.kDriveKV)      // Velocity feedforward
      .withKA(DriveConstants.kDriveKA);     // Acceleration feedforward
    
    // return the configuration
    return config;
  }

  /**
   * Gets the Neo steering motor configuration.
   */
  private SparkMaxConfig getSteerMotorConfig() {
    SparkMaxConfig config = new SparkMaxConfig();

    // Basic motor configuration (hardcoded here for safety)
    config
      .idleMode(IdleMode.kBrake)                       // Brake mode for better control
      .inverted(steerMotorInverted)                    // Set motor inversion
      .smartCurrentLimit(25)                // Continuous current limit in Amps
      .secondaryCurrentLimit(35, 20)  // Peak current limit: 35A for 20 chop cycles (0.4s) before limiting to 25A
      .voltageCompensation(12.0);       // Enable voltage compensation at 12V

    // Closed-loop PID configuration (gains are tunable in constants)
    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .outputRange(-1.0, 1.0)
      .positionWrappingEnabled(true)      
      .positionWrappingInputRange(
        -DriveConstants.kSteerGearRatio / 2.0, 
         DriveConstants.kSteerGearRatio / 2.0
      )
      .p(DriveConstants.kSteerKP)
      .i(DriveConstants.kSteerKI)
      .d(DriveConstants.kSteerKD)
      .feedForward
        .kS(DriveConstants.kSteerKS)
        .kV(DriveConstants.kSteerKV)
        .kA(DriveConstants.kSteerKA);

    // return the configuration
    return config;
  }

  /**
   * Called by the drive subsystem's periodic() method
   */
  public void periodic() {
    // Get the current steer angle once each periodic (shared by state and position)
    Rotation2d currentAngle = Rotation2d.fromRadians(getSteerAngleRadians());

    // Refresh the drive motor status signals (avoids newing StatusSignal each time)
    BaseStatusSignal.refreshAll(drivePosition, driveVelocity);
 
    // Update cached state (avoids newing SwerveModuleState each time)
    cachedState.speedMetersPerSecond = Conversions.rotationsToMeters(
      driveVelocity.getValueAsDouble(), 
      DriveConstants.kDriveGearRatio, 
      DriveConstants.kWheelCircumference
    );
    cachedState.angle = currentAngle;

    // Update cached position (avoids newing SwerveModuleState each time)
    cachedPosition.distanceMeters = Conversions.rotationsToMeters(
      drivePosition.getValueAsDouble(),
      DriveConstants.kDriveGearRatio,
      DriveConstants.kWheelCircumference
    );
    cachedPosition.angle = currentAngle;

    // Update the absolute encoder (for diagnostics)
    absoluteEncoder.periodic();

    // Check for encoder drift and reset if necessary
    checkEncoderDrift();
  }

  /**
   * Checks for encoder drift and resets the steer encoder if necessary.
   * This is a safety feature to prevent the steer encoder from 
   * drifting too far from the absolute encoder.
   */
  private void checkEncoderDrift() {
    double volts = absoluteEncoder.getRawVoltage();
    boolean stationary = Math.abs(cachedState.speedMetersPerSecond) < 0.05 && Math.abs(getSteerVelocityRadPerSec()) < 0.5;
    double errorRad = Utils.normalizeAngleRadians(absoluteEncoder.getAngleRadians() - getSteerAngleRadians());

    if (
      stationary && 
      volts > 0.15 && 
      volts < 4.85 && 
      Math.abs(errorRad) > Units.degreesToRadians(10)
    ) {
      if (++driftLoops >= 25) { 
        resetSteerEncoders();
        driftLoops = 0;
      }
    } else {
      driftLoops = 0;
    }
  }

  /**
   * Resets the drive and steering encoders.
   */
  public void resetEncoders() {
    driveMotor.setPosition(0);
    resetSteerEncoders();
  }

  /**
   * Resets the steer encoder.
   * Steer encoder is synchronized to the absolute encoder.
   */
  public void resetSteerEncoders() {
    steerEncoder.setPosition(Conversions.radiansToRotations(
      absoluteEncoder.getAngleRadians(), 
      DriveConstants.kSteerGearRatio
    ));
  }

  /**
   * Gets the name of the swerve module
   * @return The module name
   */
  public String getName() {
    return moduleName;
  }

  /**
   * Gets the target state of the swerve module
   * @return
   */
  public SwerveModuleState getTargetState() {
    return new SwerveModuleState(targetState.speedMetersPerSecond, targetState.angle);
  }

  /**
   * Gets the most recent cached state of the swerve module
   * Updated in periodic() to help with optimization
   * @return Defensive copy of cached SwerveModuleState
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(cachedState.speedMetersPerSecond, cachedState.angle);
  }

  /**
   * Gets the most recent cached position of the swerve module
   * Updated in periodic() to help with optimization
   * @return Defensive copy of cached SwerveModulePosition
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(cachedPosition.distanceMeters, cachedPosition.angle);
  }

  /**
   * Sets the desired state of the swerve module
   * @param desiredState The desired SwerveModuleState
   * @param feedforward The feedforward force to apply
   */
  public void setDesiredState(SwerveModuleState desiredState, Force feedforward) {
    // Bad input: stop driving, leave the steer setpoint alone
    if (
      desiredState == null ||
      desiredState.angle == null ||
      !Double.isFinite(desiredState.speedMetersPerSecond) ||
      !Double.isFinite(desiredState.angle.getRadians())
    ) {
      setDriveVelocity(0.0, null);
      return;
    }

    // Command the robot to move
    setDriveVelocity(desiredState.speedMetersPerSecond, feedforward);
    setSteerAngle(desiredState.angle.getRadians());
  }

  /**
   * Sets the drive motor velocity adjusted for gear ratio and wheel circumference
   * @param velocityMPS Desired velocity in m/s
   * @param feedforward The feedforward force to apply
   */
  private void setDriveVelocity(double velocityMPS, Force feedforward) {
    // Clamp the velocity to a safe range
    double clamped = MathUtil.clamp(
      velocityMPS, 
      -DriveConstants.kMaxSpeedMetersPerSecond, 
      DriveConstants.kMaxSpeedMetersPerSecond
    );

    // Update the target state for telemetry
    targetState.speedMetersPerSecond = clamped;

    // Convert velocity (m/s) to motor rotations per second (RPS)
    double velocityRPS = Conversions.metersToRotations(
      clamped, 
      DriveConstants.kDriveGearRatio, 
      DriveConstants.kWheelCircumference
    );

    // Convert force (N) to voltage
    double feedforwardVolts = 0.0;
    if (feedforward != null) {
      double volts = feedforward.in(Newtons) * DriveConstants.kDriveVoltsPerNewton;
      if (Double.isFinite(volts)) {
        feedforwardVolts = MathUtil.clamp(volts, -12.0, 12.0);
      }
    }

    // Command the drive motor with velocity and feedforward
    driveMotor.setControl(driveVelocityRequest.withVelocity(velocityRPS).withFeedForward(feedforwardVolts));
  }

  /**
   * Sets the steering angle in radians, normalized to the range (-π, π)
   * Public for sysId or testing, but should normally be called by setDesiredState()
   * @param angleRadians Desired angle in radians
   */
  public void setSteerAngle(double angleRadians) {
    // Ignore bad input
    if (!Double.isFinite(angleRadians)) {
      return;
    }
    
    // Update the target state for telemetry
    targetState.angle = Rotation2d.fromRadians(angleRadians);

    // Convert the angle to motor rotations
    double rotations = Conversions.radiansToRotations(
      Utils.normalizeAngleRadians(angleRadians), 
      DriveConstants.kSteerGearRatio
    );

    // Command the steering motor to the desired angle
    steerPIDController.setSetpoint(rotations, SparkMax.ControlType.kPosition);
  }

  /**
   * Set both drive and steering motor motor to brake/coast mode
   * @param brake True for brake mode, false for coast mode
   */
  public void setMotorBrake(boolean brake) {
    // Set drive motor neutral/idle mode
    driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    
    // Set steering motor neutral/idle mode
    SparkMaxConfig brakeConfig = new SparkMaxConfig();
    brakeConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);

    // Apply the configuration to the steering motor
    SparkMaxFactory.applyConfig(
      steerMotor, 
      brakeConfig, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters
    );
  }

  /**
   * Stops both drive and steering motors
   */
  public void stop() {
    driveMotor.stopMotor();
    steerMotor.stopMotor();
  }

  // ---------------------------------------------------------------------------------------
  // Control methods used by SysId or testing (not used by main drive code)
  // ---------------------------------------------------------------------------------------

  /**
   * Gets the current drive position in meters
   * @return The drive position in meters
   */
  public double getDrivePositionMeters() { 
    return cachedPosition.distanceMeters; 
  }

  /**
   * Gets the current drive velocity in meters per second
   * @return The drive velocity in meters per second
   */
  public double getDriveVelocityMPS() { 
    return cachedState.speedMetersPerSecond; 
  }

  /**
   * Gets the current drive voltage in volts
   * @return The drive voltage in volts
   */
  public double getDriveVoltage() { 
    return driveMotorVoltage.refresh().getValueAsDouble(); 
  }

  /**
   * Sets the drive motor voltage directly (for sysId or testing)
   * @param volts Desired voltage in volts
   */
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(new VoltageOut(volts));
  }

  /**
   * Gets the current steering angle in radians (normalized to (-π, π))
   * @return The steering angle in radians
   */
  public double getSteerAngleRadians() {
    return Utils.normalizeAngleRadians(Conversions.rotationsToRadians(steerEncoder.getPosition(), DriveConstants.kSteerGearRatio));
  }

  /**
   * Gets the current steering position in radians (unwrapped)
   * @return The steering position in radians
   */
  public double getSteerPositionRadians() {
    return Conversions.rotationsToRadians(steerEncoder.getPosition(), DriveConstants.kSteerGearRatio);
  }

  /**
   * Gets the current steering velocity in radians per second
   * @return The steering velocity in radians per second
   */
  public double getSteerVelocityRadPerSec() {
    return Conversions.rotationsToRadians(steerEncoder.getVelocity() / 60.0, DriveConstants.kSteerGearRatio);
  }

  /**
   * Gets the current steering voltage in volts
   * @return The steering voltage in volts
   */
  public double getSteerVoltage() {
    return steerMotor.getAppliedOutput() * steerMotor.getBusVoltage();
  }

  /**
   * Sets the steering motor voltage directly (for sysId or testing)
   * @param volts Desired voltage in volts
   */
  public void setSteerVoltage(double volts) {
    steerMotor.setVoltage(volts);
  }
  
  /**
   * Initialize the data sent to SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Target Speed (mps)", () -> Utils.showDouble(targetState.speedMetersPerSecond), null);
    builder.addDoubleProperty("Target Angle (deg)", () -> Utils.showDouble(targetState.angle.getDegrees()), null);
    builder.addDoubleProperty("Current Speed (mps)", () -> Utils.showDouble(cachedState.speedMetersPerSecond), null);
    builder.addDoubleProperty("Current Angle (deg)", () -> Utils.showDouble(cachedState.angle.getDegrees()), null);
    builder.addDoubleProperty("Drive Voltage (V)", () -> Utils.showDouble(driveMotorVoltage.refresh().getValueAsDouble()), null);
    builder.addDoubleProperty("Drive Current (A)", () -> Utils.showDouble(driveSupplyCurrent.refresh().getValueAsDouble()), null);
    builder.addDoubleProperty("Drive Temp (deg C)", () -> Utils.showDouble(driveTemp.refresh().getValueAsDouble()), null);
    builder.addDoubleProperty("Steer Voltage (V)", () -> Utils.showDouble(getSteerVoltage()), null);
    builder.addDoubleProperty("Steer Current (A)", () -> Utils.showDouble(steerMotor.getOutputCurrent()), null);
    builder.addDoubleProperty("Steer Temp (deg C)", () -> Utils.showDouble(steerMotor.getMotorTemperature()), null);
  }
}
