// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Newtons;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants.AnalogConstants;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.Utils;

/**
 * Swerve drive subsystem using YAGSL.
 * 
 * Integrated functionality:
 * 1. PathPlanner 
 * 2. PhotonVision integration via VisionSubsystem
 * 3. Slew rate limiting for smoother joystick control
 */
public class DriveSubsystem extends SubsystemBase {
  // Gyroscope
  private final Navx2Gyro gyro;

  // Swerve modules - ensure indexed order is 0 = FL, 1 = FR, 2 = BL, 3 = BR
  private final DriveModule[] modules;

  // Kinematics and odometry
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator poseEstimator;

  // Setpoint generator
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint driveSetpoint;

  // Field visualization
  private final Field2d field2d = new Field2d();

  // Cached zero feedforward for when the input speeds are invalid or feedforwards are unusable
  private static final DriveFeedforwards kZeroFeedforwards = DriveFeedforwards.zeros(4);

  // Slew rate limiters for joystick inputs to prevent sudden changes in speed
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRateLimit);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTranslationSlewRateLimit);
  private final SlewRateLimiter rLimiter = new SlewRateLimiter(DriveConstants.kRotationSlewRateLimit);

  // Current robot state
  private boolean fieldRelative = true;
  private boolean slowMode = false;
  private boolean brakeMode = true;
  private long acceptedVisionCount = 0;
  private long rejectedVisionCount = 0;
  private long blurRejectedVisionCount = 0;
  private long consecutiveRejectedJumps = 0;

  // Cache pose info during alignment in disabledPeriodic()
  private String lastAlignAutoName = null;
  private Pose2d cachedStartingPose = null;

  // Alliance that the pose heading was last seeded for (empty = not seeded yet)
  private Optional<Alliance> seededAlliance = Optional.empty();

  // SysId logging variables to avoid creating new objects every loop
  private final MutVoltage sysIdVoltage = Volts.mutable(0);
  private final MutDistance sysIdDistance = Meters.mutable(0);
  private final MutLinearVelocity sysIdLinVel = MetersPerSecond.mutable(0);
  private final SysIdRoutine driveSysId;

  /**
   * Creates a new swerve DriveSubsystem using the YAGSL library.
   * @param directory The directory containing the swerve drive JSON configuration
   */
  public DriveSubsystem() {
    // Create the gyro
    gyro = new Navx2Gyro();

    // Short delay so the module encoders initialize properly before we read them for the first time
    Timer.delay(0.1);

    // Initialize swerve modules
    modules = new DriveModule[] {
      new DriveModule(
        "FL",
        CANConstants.kFrontLeftDriveID,
        CANConstants.kFrontLeftSteerID,
        DriveConstants.kFrontLeftDriveInverted,
        DriveConstants.kFrontLeftSteerInverted,
        AnalogConstants.kFrontLeftAbsEncoderID,
        DriveConstants.kFrontLeftAbsEncoderOffsetRadians,
        DriveConstants.kFrontLeftAbsEncoderInverted
      ),
      new DriveModule(
        "FR",
        CANConstants.kFrontRightDriveID,
        CANConstants.kFrontRightSteerID,
        DriveConstants.kFrontRightDriveInverted,
        DriveConstants.kFrontRightSteerInverted,
        AnalogConstants.kFrontRightAbsEncoderID,    
        DriveConstants.kFrontRightAbsEncoderOffsetRadians,
        DriveConstants.kFrontRightAbsEncoderInverted
      ),
      new DriveModule(
        "BL",
        CANConstants.kBackLeftDriveID,
        CANConstants.kBackLeftSteerID,
        DriveConstants.kBackLeftDriveInverted,
        DriveConstants.kBackLeftSteerInverted,
        AnalogConstants.kBackLeftAbsEncoderID,
        DriveConstants.kBackLeftAbsEncoderOffsetRadians,
        DriveConstants.kBackLeftAbsEncoderInverted
      ),
      new DriveModule(
        "BR",
        CANConstants.kBackRightDriveID,
        CANConstants.kBackRightSteerID,
        DriveConstants.kBackRightDriveInverted,
        DriveConstants.kBackRightSteerInverted,
        AnalogConstants.kBackRightAbsEncoderID,
        DriveConstants.kBackRightAbsEncoderOffsetRadians,
        DriveConstants.kBackRightAbsEncoderInverted
      )
    };
    for (var module : modules) {
      module.periodic();
    }

    // Initialize swerve drive kinematics
    kinematics = new SwerveDriveKinematics(DriveConstants.kSwerveModuleTranslations);
    
    // Initialize pose estimator for vision integration
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,                               // The kinematics object
      gyro.getAngle(),                          // The current gyro angle
      getModulePositions(),                     // The current module positions
      new Pose2d(),                             // Will be updated by auto and vision      
      VecBuilder.fill(0.1, 0.1, 0.1),  // State std devs (x, y, heading)
      VecBuilder.fill(0.9, 0.9, 0.9)   // Vision std devs (increase these if vision is jumpy)
    );

    // Initialize the swerve setpoint generator
    setpointGenerator = new SwerveSetpointGenerator(
      DriveConstants.kRobotConfig, // The robot configuration. This is the same config used for generating trajectories and running path following commands.
      DriveConstants.kSteerMaxAngularSpeedRadsPerSecond // The max rotation velocity of a swerve module steer motor in radians per second.
    );
    
    // Initialize the drive setpoint
    driveSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates(), kZeroFeedforwards);

    // Configure AutoBuilder for path following
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier
      this::driveRobotRelative, // Method that will drive the robot given ChassisSpeeds & feedforwards
      new PPHolonomicDriveController(
        new PIDConstants(5.0, 0.0, 0.0), // Translation
        new PIDConstants(5.0, 0.0, 0.0)  // Rotation
      ),
      DriveConstants.kRobotConfig, // Robot configuration
      Utils::isRedAlliance, // Method to flip path based on alliance color
      this // Reference to this subsystem to set requirements
    );

    // Configure SysId routine for drive characterization
    driveSysId = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(1.0), Volts.of(7.0), Seconds.of(10.0)),
      new SysIdRoutine.Mechanism(
        volts -> {
          for (var m : modules) {
            m.setSteerAngle(0.0); // hold straight
            m.setDriveVoltage(volts.in(Volts));
          }
        },
        log -> {
          for (int i = 0; i < 4; i++) {
            log.motor("drive-" + modules[i].getName())
              .voltage(sysIdVoltage.mut_replace(modules[i].getDriveVoltage(), Volts))
              .linearPosition(sysIdDistance.mut_replace(modules[i].getDrivePositionMeters(), Meters))
              .linearVelocity(sysIdLinVel.mut_replace(modules[i].getDriveVelocityMPS(), MetersPerSecond));
          }
        },
        this
      )
    );

    // Add data to dashboard
    SmartDashboard.putData("Drive", this);
    SmartDashboard.putData("Drive/Field", field2d);

    // Output initialization progress
    Utils.logInfo("Drive subsystem initialized");
  }

  @Override
  public void periodic() {
    // Run the gyro's periodic method
    gyro.periodic();

    // Call each swerve module's periodic before updating the pose estimate since 
    // they calculate and cache their state/position in their periodics and the 
    // pose estimator uses the cached positions in its update.
    for (var module : modules) {
      module.periodic();
    }

    // Update odometry
    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), gyro.getAngle(), getModulePositions());
        
    // Update field visualization
    field2d.setRobotPose(getPose());
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------

  /**
   * Drive the robot using the given robot-relative chassis speeds.
   * @param speeds The desired robot-relative speeds
   * @param feedforwards Optional module feedforward forces (e.g. from PathPlanner).
   *                     If null, feedforward is computed by the setpoint generator (or omitted).
   */
  private void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
    // Drive with setpoint generator if necessary
    if (DriveConstants.kUseSetpointGenerator) {
      driveWithSetpointGenerator(speeds);
      return;
    }

    // Reject non-finite input before it reaches either path. Bad speeds also
    // invalidate any feedforwards that came with them.
    boolean valid = isValidSpeeds(speeds);
    if (!valid) {
      speeds = new ChassisSpeeds();
    }

    // Fall back to zero feedforward if the speeds were bad or the forces are unusable
    DriveFeedforwards ff = (valid && isValidFeedforwards(feedforwards)) ? feedforwards : kZeroFeedforwards;
    Force[] desiredForces = ff.linearForces();

    // Compensate for skew when translating and rotating simultaneously
    speeds = ChassisSpeeds.discretize(speeds, DriveConstants.kPeriodicTimeSeconds);

    // Compute the desired swerve module states from the chassis speeds
    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);

    // DesaturateWheelSpeeds scales every module by the same factor, so compute
    // that factor up front and apply it to the feedforward forces as well
    double maxRequested = 0.0;
    for (SwerveModuleState s : desiredStates) {
      maxRequested = Math.max(maxRequested, Math.abs(s.speedMetersPerSecond));
    }
    double desatScale = maxRequested > DriveConstants.kMaxSpeedMetersPerSecond
      ? DriveConstants.kMaxSpeedMetersPerSecond / maxRequested
      : 1.0;

    // Desaturate the wheel speeds to ensure they are within the maximum allowed speed
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // Set each swerve module state directly from the next calculated setpoint
    for (int i = 0; i < 4; i++) {
      Rotation2d current = modules[i].getState().angle;
      SwerveModuleState s = desiredStates[i];

      // Determine if the voltage sign should be flipped based on the angle difference. 
      // If the angle difference is greater than 90 degrees, flip the voltage sign.
      // Must be done before calling optimize() since optimize() may change the 
      // angle to be more than 90 degrees away from the current angle.
      boolean flipped = Math.abs(s.angle.minus(current).getDegrees()) > 90.0;

      // Optimize the desired state to minimize rotation and avoid unnecessary wheel spinning
      s.optimize(current);

      // Scale the speed by the cosine of the angle difference to reduce  
      // speed when the wheel is not aligned with the desired direction
      s.cosineScale(current);

      // Feedforward follows the same desaturation, flip, and cosine scaling as the speed
      double scale = desatScale * (flipped ? -1.0 : 1.0) * s.angle.minus(current).getCos();

      // Set the desired state and feedforward for the current module, 
      // applying the calculated scale to the feedforward force
      modules[i].setDesiredState(s, desiredForces[i].times(scale));
    }
  }
  
  /**
   * Drives the robot using a SwerveSetpoint, which includes 
   * both desired chassis speeds and feedforward forces.
   * @param setpoint The desired swerve setpoint
   */
  private void driveWithSetpointGenerator(ChassisSpeeds speeds) {
    // Skip if the setpoint generator is disabled
    if (!DriveConstants.kUseSetpointGenerator) {
      return;
    }

    // Reject non-finite input before it reaches either path
    if (!isValidSpeeds(speeds)) {
      speeds = new ChassisSpeeds();
    }

    // Create a new drive setpoint based on the current drive setpoint & the desired chassis speeds.
    driveSetpoint = setpointGenerator.generateSetpoint(driveSetpoint, speeds, DriveConstants.kPeriodicTimeSeconds);
    
    // If the generator produced garbage, reset from the real module states and skip this cycle
    if (!isValidSpeeds(driveSetpoint.robotRelativeSpeeds())) {
      SwerveModuleState[] stopped = getModuleStates();
      for (var s : stopped) {
        s.speedMetersPerSecond = 0.0;
      }
      driveSetpoint = new SwerveSetpoint(new ChassisSpeeds(), stopped, kZeroFeedforwards);
    }

    // Get the setpoint's calculated swerve module states for each module
    SwerveModuleState[] desiredStates = driveSetpoint.moduleStates();

    // Get the setpoint's feedforward forces for each module
    Force[] desiredForces = driveSetpoint.feedforwards().linearForces();

    // Set each swerve module state directly from the next calculated setpoint
    for (int i = 0; i < 4; i++) {
      modules[i].setDesiredState(desiredStates[i], desiredForces[i]);
    }
  }

  /**
   * Validates that the given chassis speeds are finite numbers (not NaN or infinite).
   * @param speeds The robot-relative chassis speeds to validate
   * @return True if all speeds are finite, false otherwise
   */
  private boolean isValidSpeeds(ChassisSpeeds speeds) {
    if (
      speeds == null ||
      !Double.isFinite(speeds.vxMetersPerSecond) ||
      !Double.isFinite(speeds.vyMetersPerSecond) ||
      !Double.isFinite(speeds.omegaRadiansPerSecond)
    ) {
      return false;
    }
    return true;
  }

  /**
   * Validates that the given feedforward forces are finite numbers 
   * (not NaN or infinite) and of the correct length.
   * @param feedforwards The feedforward forces to validate
   * @return True if the feedforwards are valid, false otherwise
   */
  private static boolean isValidFeedforwards(DriveFeedforwards feedforwards) {
    if (feedforwards == null) {
      return false;
    }
    Force[] forces = feedforwards.linearForces();
    if (forces == null || forces.length != 4) {
      return false;
    }
    for (Force f : forces) {
      if (!Double.isFinite(f.in(Newtons))) {
        return false;
      }
    }
    return true;
  }

  /**
   * Set each swerve module to brake/coast mode
   * @param brake True to enable motor brake, false for coast
   */
  private void setMotorBrake(boolean brake) {
    if (brake == this.brakeMode) return;
    this.brakeMode = brake;
    for (var module : modules) {
      module.setMotorBrake(brakeMode);
    }
  }

  /**
   * Resets all swerve module encoders to zero. Should be called when the robot is stationary and
   * the wheels are in a known position (e.g. straight forward) to ensure accurate odometry.
   */
  private void resetEncoders() {
    for (var module : modules) {
      module.resetEncoders();
    }
  }

  /**
   * Resets odometry to a specific pose and resets module encoders.
   * NOTE: Should never need to call this if vision is working properly.
   */
  private void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(
      gyro.getAngle(), 
      getModulePositions(), 
      pose == null ? getSeedPose() : pose
    );
  }

  /**
   * Resets ONLY the heading of the pose estimate to the alliance-forward direction, 
   * keeping the estimated translation. Use when the driver points the robot away 
   * from their driver station and needs to re-establish field-relative "forward".
   * The alliance is read when this runs, not when the command is built.
   */
  private void resetHeading() {
    poseEstimator.resetPosition(
      gyro.getAngle(), 
      getModulePositions(), 
      new Pose2d(getPose().getTranslation(), getSeedPose().getRotation())
    );
  }

  /**
   * Seeds the pose heading to the alliance-forward direction once per alliance. The alliance is
   * often unknown when the drive subsystem is constructed and can change between matches without
   * a code restart, so this runs from disabledPeriodic() (and the auto/teleop init as a backstop).
   * It is a no-op until the alliance is known and after the heading was seeded for that alliance,
   * so it never fights vision or the driver's manual heading reset. Assumes the robot is placed
   * facing away from its driver station, as at the start of a match.
   */
  public void seedHeadingForAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.equals(seededAlliance)) {
      return;
    }

    // In sim there is no real position to preserve, so seed the whole pose.
    // On a real robot, only do this if vision hasn't given us a position yet.
    if (RobotBase.isSimulation() || acceptedVisionCount == 0) {
      // seed the whole pose
      resetOdometry(getSeedPose());
    } else {
      // reads the current alliance but keeps the translation estimate
      resetHeading();
    }
    
    // cache the alliance so we don't re-seed until it changes
    seededAlliance = alliance;
  }

  /**
   * Gets the current heading of the robot from the pose estimator.
   * This is what should be fed into the drive (auto and teleop)
   * functions when calculating chassis speeds & module states.
   * @return Current heading as a Rotation2d
   */
  private Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Gets the default starting pose based on alliance color.
   * This is used to seed the pose estimator at the start of a match if no vision is available.
   * @return The default starting Pose2d
   */
  private Pose2d getSeedPose() {    
    return Utils.isRedAlliance() ? DriveConstants.kRedSeedPose : DriveConstants.kBlueSeedPose;
  }

  /**
   * Gets the current pose of the robot as measured by odometry and vision fusion.
   * @return Current pose 
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the current robot-relative chassis speeds
   * @return Current robot-relative ChassisSpeeds
   */
  private ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Gets the current field-relative chassis speeds
   * @return Current field-relative ChassisSpeeds
   */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeSpeeds(), getHeading());
  }

  /**
   * Gets the current module states
   * @return Array of current SwerveModuleStates
   */
  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      modules[0].getState(),  // front left
      modules[1].getState(),  // front right
      modules[2].getState(),  // back left
      modules[3].getState()   // back right
    };
  }

  /**
   * Gets the current module positions
   * @return Array of current SwerveModulePositions
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      modules[0].getPosition(), // front left
      modules[1].getPosition(), // front right
      modules[2].getPosition(), // back left
      modules[3].getPosition()  // back right
    };
  }

  /**
   * Gets whether the gyro is connected and functional
   * @return True if the gyro is connected and functional, false otherwise
   */
  private boolean isGyroConnected() {
    return gyro != null && gyro.isConnected();
  }

  /**
   * Gets whether field-relative driving is enabled
   * @return Field-relative status
   */
  private boolean isFieldRelative() {
    return fieldRelative && isGyroConnected();
  }

  /**
   * Gets whether slow mode is enabled
   * @return Slow mode status
   */
  private boolean isSlowMode() {
    return slowMode;
  }
  
  /**
   * Gets whether brake mode is enabled
   * @return Brake mode status
   */
  private boolean isBrakeMode() {
    return brakeMode;
  }

  /**
   * Calculates the distance from the current pose to a target pose.
   * @param target The target to calculate distance to
   * @return The distance to the target pose
   */
  private double distanceTo(Pose2d targetPose) {
    return getPose().getTranslation().getDistance(targetPose.getTranslation());
  }

  // ---------------------------------------------------------------------------------------
  // Public triggers that expose private state
  // ---------------------------------------------------------------------------------------

  public final Trigger isFieldRelativeTrigger = new Trigger(this::isFieldRelative)
    .debounce(0.05, DebounceType.kBoth);
    
  public final Trigger isSlowModeTrigger = new Trigger(this::isSlowMode)
    .debounce(0.05, DebounceType.kBoth);
    
  public final Trigger isBrakeModeTrigger = new Trigger(this::isBrakeMode)
    .debounce(0.05, DebounceType.kBoth);

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match (autonomous, teleop, post match)
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the drive subsystem at the start of the autonomous phase.
   * Should be called from Robot.autonomousInit() or a command scheduler binding.
   */
  public void autonomousInit() {
    // Never start a match in coast (pit toggle may have been left on)
    if (!brakeMode) {
      setMotorBrake(true);
    }

    // Backstop in case the alliance only became known after the last disabledPeriodic()
    seedHeadingForAlliance();

    // Reset drive setpoint for setpoint generator
    driveSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates(), kZeroFeedforwards);

    // Reset slew rate limiters
    xLimiter.reset(0.0);
    yLimiter.reset(0.0);
    rLimiter.reset(0.0);
    
    // Reset state variables
    fieldRelative = true;
    slowMode = false;

    // Log initialization
    Utils.logInfo("Drive subsystem initialized for autonomous");
  }

  /**
   * Initializes the drive subsystem at the start of the teleop phase.
   * Should be called from Robot.teleopInit() or a command scheduler binding.
   */
  public void teleopInit() {
    // Never start a match in coast (pit toggle may have been left on)
    if (!brakeMode) {
      setMotorBrake(true);
    }

    // Backstop (no-op if already seeded for this alliance, e.g. after autonomous)
    seedHeadingForAlliance();

    // Reset drive setpoint for setpoint generator
    driveSetpoint = new SwerveSetpoint(new ChassisSpeeds(), getModuleStates(), kZeroFeedforwards);

    // Reset slew rate limiters
    xLimiter.reset(0.0);
    yLimiter.reset(0.0);
    rLimiter.reset(0.0);
    
    // Reset state variables
    fieldRelative = true;
    slowMode = false;
  
    // Log initialization
    Utils.logInfo("Drive subsystem initialized for teleop");
  }

  /**
   * Initializes the drive subsystem for post match (disabled) state.
   * Should be called from Robot.postMatch() or a command scheduler binding.
   */
  public void postMatch() {
    // Set motors to brake mode for match (do this before resetting encoders)
    setMotorBrake(true);
  
    // Reset slew rate limiters
    xLimiter.reset(0.0);
    yLimiter.reset(0.0);
    rLimiter.reset(0.0);

    // Log initialization
    Utils.logInfo("Drive subsystem initialized for post match");
  }

  /**
   * Add a vision measurement to the pose estimator with validation and filtering.
   * This method can be called by the VisionSubsystem whenever a new vision 
   * measurement is available.
   * @param visionPose The vision pose to add
   * @param timestamp The timestamp of the vision measurement
   * @param stdDevs The standard deviations of the vision measurement
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp, double[] stdDevs) {
    // Get current time
    double now = Timer.getFPGATimestamp();

    try {      
      // Validate components
      if (visionPose == null || stdDevs == null || stdDevs.length < 3) {
        rejectedVisionCount++;
        return;
      }

      // Reject timestamps older than 0.3 seconds
      if ((now - timestamp) > DriveConstants.kVisionMeasurementMaxAge) {
        rejectedVisionCount++;
        return;
      }

      // Reject timestamps from the future
      if (timestamp > now) {
        rejectedVisionCount++;
        return;
      }

      // Reject measurements taken while moving or rotating fast (motion blur degrades tag corners).
      // Uses measured (not commanded) robot-relative speeds from the modules.
      ChassisSpeeds speeds = getRobotRelativeSpeeds();
      double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
      double angularSpeed = Math.abs(speeds.omegaRadiansPerSecond);
      if (
        angularSpeed > DriveConstants.kVisionMaxAngularVelocityRadPerSec ||
        linearSpeed > DriveConstants.kVisionMaxLinearVelocityMetersPerSec
      ) {
        blurRejectedVisionCount++;
        return;
      }

      // Reject large translation jumps, UNLESS:
      //  - we haven't accepted any vision yet (odometry-only pose may already be stale), or
      //  - we've rejected several in a row (likely genuine drift, not a bad reading)
      double translationDistance = getPose().getTranslation().getDistance(visionPose.getTranslation());
      boolean isJump = translationDistance > DriveConstants.kVisionMaxTranslationJumpMeters;
      boolean shouldGateJump = isJump
        && acceptedVisionCount > 0
        && consecutiveRejectedJumps < DriveConstants.kMaxConsecutiveVisionRejections;

      if (shouldGateJump) {
        rejectedVisionCount++;
        consecutiveRejectedJumps++;
        return;
      }

      // If we make it here => add vision measurement to pose estimator
      poseEstimator.addVisionMeasurement(
        visionPose,
        timestamp,
        VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2])
      );

      // Increment the count of accepted vision measurements for monitoring purposes
      acceptedVisionCount++;
      consecutiveRejectedJumps = 0;
    } catch (Exception ex) {
      Utils.logError("Error adding vision measurement: " + ex.getMessage());
    }
  }

  /**
   * Publishes guidance to help the drive team manually place the robot at the
   * starting pose of the currently selected auto. Call this repeatedly from
   * Robot.disabledPeriodic() so it updates live as the robot gets nudged into
   * place.
   *
   * Requires a valid, alliance-consistent current pose estimate (e.g. seeded
   * by AprilTag vision) -- if getPose() isn't meaningful yet, neither is this.
   *
   * @param autoNameSupplier Supplies the name of the currently selected auto (e.g. autoChooser::getSelected)
   */
  public void publishStartingPoseAlignment(Supplier<String> autoNameSupplier) {
    // Get the currently selected auto name from the supplier
    String autoName = autoNameSupplier.get();
    if (autoName == null || autoName.isEmpty()) {
      SmartDashboard.putString("Auto Align/Status", "No auto selected");
      return;
    }

    // If the auto name has changed since last time, load & cache the new starting pose
    if (!autoName.equals(lastAlignAutoName)) {
      lastAlignAutoName = autoName;
      try {
        // Starting pose is always relative to a blue alliance origin
        cachedStartingPose = new PathPlannerAuto(autoName).getStartingPose();
        SmartDashboard.putString("Auto Align/Status", "Ready");
      } catch (Exception ex) {
        cachedStartingPose = null;
        SmartDashboard.putString("Auto Align/Status", "Couldn't load starting pose for \"" + autoName + "\"");
        return;
      }
    }

    // If we don't have a valid starting pose, we can't provide guidance
    if (cachedStartingPose == null) {
      return;
    }

    // Flip the starting pose for red alliance if necessary
    Pose2d targetPose = Utils.isRedAlliance()
        ? FlippingUtil.flipFieldPose(cachedStartingPose)
        : cachedStartingPose;

    // Get the current pose of the robot from the pose estimator (odometry + vision)
    Pose2d currentPose = getPose();

    // Transform from current pose to target pose, in the ROBOT's current frame:
    // +X = need to move forward, +Y = need to move left,
    // +rotation = need to rotate CCW
    Transform2d error = targetPose.minus(currentPose);
    double forwardMeters = error.getX();
    double leftMeters = error.getY();
    double rotateDegrees = error.getRotation().getDegrees();
    double totalErrorMeters = error.getTranslation().getNorm();

    SmartDashboard.putString("Auto Align/Selected Auto", autoName);
    SmartDashboard.putString("Auto Align/Move", String.format(
        "%s %.2fm, %s %.2fm, %s %.1f°",
        forwardMeters >= 0 ? "Fwd" : "Back", Math.abs(forwardMeters),
        leftMeters >= 0 ? "Left" : "Right", Math.abs(leftMeters),
        rotateDegrees >= 0 ? "CCW" : "CW", Math.abs(rotateDegrees)));
    SmartDashboard.putBoolean("Auto Align/In Position",
        totalErrorMeters < DriveConstants.kStartPoseTranslationToleranceMeters
        && Math.abs(rotateDegrees) < DriveConstants.kStartPoseRotationToleranceDegrees);

    // Optional: overlay both poses on a Field2d for a visual "you are here" /
    // "target" view on Shuffleboard/Glass, if you already keep one around
    field2d.setRobotPose(targetPose);
  }

  // ----------------------------------------------------------------------------------------
  // SysId Command Factories
  // ----------------------------------------------------------------------------------------

  /**
   * Creates a command to run the SysId drive characterization routine.
   * @param d The direction of the characterization (forward, backward, left, right)
   * @return A command that runs the SysId routine
   */
  public Command sysIdDriveQuasistatic(SysIdRoutine.Direction d) { 
    return driveSysId.quasistatic(d); 
  }    
  
  /**
   * Creates a command to run the SysId drive characterization routine.
   * @param d The direction of the characterization (forward, backward, left, right)
   * @return A command that runs the SysId routine
   */
  public Command sysIdDriveDynamic(SysIdRoutine.Direction d) { 
    return driveSysId.dynamic(d); 
  }

  // ---------------------------------------------------------------------------------------
  // Public Command Factory Methods
  // ---------------------------------------------------------------------------------------

  /**
   * Drive the robot using the raw joystick inputs.
   * This function applies deadbands, shaping, and slow mode scaling. 
   * Joystick inputs are converted to robot chassis speeds in m/s and rad/s.
   * Field relative driving is automatically disabled if the gyro is disconnected. 
   * @param xSpeedSupplier Speed in x direction (-1 to 1)
   * @param ySpeedSupplier Speed in y direction (-1 to 1)
   * @param rSpeedSupplier Rotation speed (-1 to 1)
   */
  public Command driveCommand(
    DoubleSupplier xSpeedSupplier, 
    DoubleSupplier ySpeedSupplier, 
    DoubleSupplier rSpeedSupplier
  ) {
    // Return a command that runs the drive logic
    return run(() -> {
      // Get raw joystick inputs from the suppliers every cycle
      double rawX = xSpeedSupplier.getAsDouble();
      double rawY = ySpeedSupplier.getAsDouble();
      double rawR = rSpeedSupplier.getAsDouble();

      // Clamp the raw inputs to -1..1 to avoid any unexpected values
      rawX = MathUtil.clamp(rawX, -1.0, 1.0);
      rawY = MathUtil.clamp(rawY, -1.0, 1.0);
      rawR = MathUtil.clamp(rawR, -1.0, 1.0);
      
      double magnitude = Math.hypot(rawX, rawY);
      double xSpeed = 0;
      double ySpeed = 0;

      // Apply circular deadband for translation 
      if (magnitude > DriveConstants.kJoystickDeadband) {
        // Rescale so speed starts at 0 at the edge of the deadband
        double clippedMagnitude = MathUtil.clamp((magnitude - DriveConstants.kJoystickDeadband) / (1.0 - DriveConstants.kJoystickDeadband), 0.0, 1.0);
        
        // Apply squaring/cubing to the clipped magnitude
        double curvedMagnitude = Math.pow(clippedMagnitude, DriveConstants.kJoystickInputExponent);

        // Re-apply the direction sign to the new magnitude
        xSpeed = (rawX / magnitude) * curvedMagnitude;
        ySpeed = (rawY / magnitude) * curvedMagnitude;
      }

      // Apply 1D deadband and squaring for rotation
      double rSpeed = MathUtil.applyDeadband(rawR, DriveConstants.kJoystickDeadband);
      rSpeed = Math.copySign(Math.pow(Math.abs(rSpeed), DriveConstants.kJoystickInputExponent), rSpeed);

      // If not using the setpoint generator, apply slew rate 
      // limiting to smooth out sudden changes in speed
      if (!DriveConstants.kUseSetpointGenerator) {
        xSpeed = xLimiter.calculate(xSpeed);
        ySpeed = yLimiter.calculate(ySpeed);
        rSpeed = rLimiter.calculate(rSpeed);
      }

      // If slow mode is enabled, scale down speeds for finer control
      if (isSlowMode()) {
        xSpeed *= DriveConstants.kSlowModeScaling;
        ySpeed *= DriveConstants.kSlowModeScaling;
        rSpeed *= DriveConstants.kSlowModeScaling;
      }

      // Convert the joystick's -1..1 to m/s and rad/s
      double xSpeedMPS = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      double ySpeedMPS = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      double rSpeedRad = rSpeed * DriveConstants.kMaxAngularSpeedRadsPerSecond;

      // Create robot-relative chassis speeds
      ChassisSpeeds chassisSpeeds;
      if (isFieldRelative()) {
        // Invert translation inputs for red alliance so that pushing the joystick
        // forward always moves the robot away from the driver station
        int invert = Utils.isRedAlliance() ? -1 : 1;

        // Convert field-relative speeds to robot-relative speeds
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          xSpeedMPS * invert, 
          ySpeedMPS * invert, 
          rSpeedRad, 
          getHeading()
        );
      } else {
        // Use robot-relative speeds directly
        chassisSpeeds = new ChassisSpeeds(xSpeedMPS, ySpeedMPS, rSpeedRad);
      }

      // Drive the robot with robot-relative speeds
      driveRobotRelative(chassisSpeeds, null);
    })
    .withName("Drive_DriveManual");
  }

  /**
   * Creates a command to align the robot at a specified AprilTag. It extracts the
   * AprilTag pose from the field layout and computes a target pose for the robot
   * to drive to, with the specified offsets. The robot will face the AprilTag 
   * when it reaches the target pose.
   * Use case: Teleop auto-alignment in front of a specific "scoring" AprilTag
   * @param tagIdSupplier A supplier of the fiducial ID of the AprilTag to align with
   * @param xOffset The X offset from the tag in meters (positive is forward, negative is backward)
   * @param yOffset The Y offset from the tag in meters (positive is left, negative is right)
   * @return Command to align with the AprilTag
   */  
  public Command alignToTagCommand(
    IntSupplier tagIdSupplier, 
    DoubleSupplier xOffsetSupplier, 
    DoubleSupplier yOffsetSupplier
  ) {
    return pathfindToPoseCommand(() -> {
      int tagId = tagIdSupplier == null ? -1 : tagIdSupplier.getAsInt();
      double xOffset = xOffsetSupplier == null ? 0.0 : xOffsetSupplier.getAsDouble();
      double yOffset = yOffsetSupplier == null ? 0.0 : yOffsetSupplier.getAsDouble();

      // Ensure tagId is valid
      if (tagId < 0) {
        Utils.logError("alignToTag: invalid tagId " + tagId);
        return null;
      }

      // Get the pose of the specified AprilTag from the field layout
      Optional<Pose3d> tag = FieldConstants.kFieldLayout.getTagPose(tagId);
      if (tag.isEmpty()) {
        Utils.logError("alignToTag: tag " + tagId + " not in field layout");
        return null;
      }

      // Robot faces the tag with the specified offsets
      // Ensure the offset accounts for 1/2 the robot length + bumper
      // to avoid smashing into the tag or the wall behind it
      return tag.get().toPose2d().transformBy(new Transform2d(
        xOffset, 
        yOffset, 
        Rotation2d.k180deg
      ));
    });
  }

  /**
   * Creates a command to drive to a specified pose using PathPlanner.
   * Use case: Teleop auto-alignment or quick, simple dynamic targets.
   * @param targetSupplier A supplier of a Pose2d to drive to
   * @return Command to drive via PathPlanner to the target pose
   */
  public Command pathfindToPoseCommand(Supplier<Pose2d> targetSupplier) {
    // Deferred command to get latest pose when scheduled
    // Set.of(this) ensures driveSubsystem is required
    return Commands.defer(() -> {
      // Get the target pose from the supplier when the command is scheduled
      Pose2d targetPose = targetSupplier == null ? null : targetSupplier.get();

      // Handle null target pose
      if (targetPose == null) {
        Utils.logError("pathfindToPose: target pose is null");
        return Commands.none();
      }

      // Create a staging pose to approach the target pose from a safe distance
      Pose2d stagingPose = targetPose.transformBy(new Transform2d(
        -DriveConstants.kStagingDistanceMeters, 
        0.0, 
        Rotation2d.kZero
      ));

      // Create a command to pathfind to the staging pose, 
      // unless we are already close enough to the target pose
      Command stagingCommand = AutoBuilder.pathfindToPose(
        stagingPose, 
        DriveConstants.kStagingConstraints, 
        DriveConstants.kStagingEndVelocityMPS
      ).unless(() -> distanceTo(targetPose) < DriveConstants.kStagingDistanceMeters + 0.3);

      // Create a command to pathfind precisely to the target pose
      Command preciseCommand = AutoBuilder.pathfindToPose(
        targetPose, 
        DriveConstants.kPrecisionConstraints, 
        0.0
      );

      // Chain the staging and precise commands together with a timeout
      return stagingCommand
        .andThen(preciseCommand)
        .withTimeout(DriveConstants.kAlignTimeoutSeconds)
        .withName("Drive_PathfindToPose");
    }, Set.of(this));
  }

  /**
   * Creates a command to follow a pre-defined PathPlanner path by name.
   * The robot should already be at the starting pose of the path when this command is scheduled.
   * Use case: Fixed, predictable paths (usually in the Autonomous period)
   * @param pathName The name of the PathPlanner path to follow (without the .path extension)
   * @return Command to follow the specified PathPlanner path
   */
  public Command followPathCommand(String pathName) {
    try {
      // PathConstraints are defined in the path file / GUI
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName))
        .withName("Drive_FollowPath_" + pathName);
    } catch (Exception ex) {
      Utils.logError("Drive_FollowPath_" + pathName + ": " + ex.getMessage());
      return Commands.none();
    }
  }

  /**
   * Creates a command to pathfind from current pose to a PathPlanner path and then follow it.
   * Use case: Entering a complex, multi-step autonomous sequence from an unpredictable spot.
   * @param pathName The name of the PathPlanner path to find then follow (without the .path extension)
   * @return Command to find then follow the specified PathPlanner path
   */
  public Command pathfindThenFollowPath(String pathName) {
    try {
      // use the staging constraints for the pathfinding portion, 
      // then follow the path with its own constraints
      return AutoBuilder.pathfindThenFollowPath(
        PathPlannerPath.fromPathFile(pathName), 
        DriveConstants.kStagingConstraints
      )
      .withName("Drive_PathfindThenFollowPath_" + pathName);
    } catch (Exception ex) {
      Utils.logError("Drive_PathfindThenFollowPath_ " + pathName + ": " + ex.getMessage());
      return Commands.none();
    }
  }

  /**
   * Reset the swerver module encoders to zero, which is useful 
   * for calibration and ensuring accurate odometry.
   */
  public Command resetEncodersCommand() {
    return runOnce(this::resetEncoders)
      .ignoringDisable(true)
      .withName("Drive_ResetEncoders");
  }

  /**
   * Reset only the robot heading to face away from the alliance driver station,
   * leaving the translation estimate (vision/odometry) untouched.
   */
  public Command resetHeadingCommand() {
    return runOnce(this::resetHeading)
      .ignoringDisable(true)
      .withName("Drive_ResetHeading");
  }

  /**
   * Reset the odometry to origin (0,0,0) with module encoder reset.
   * NOTE: Should never need to call this if vision is working properly.
   */
  public Command resetOdometryCommand() {
    // Evaluate the default pose when the command RUNS - the alliance is usually not 
    // known yet when bindings are created in the RobotContainer constructor.
    return runOnce(() -> resetOdometry(getSeedPose()))
      .ignoringDisable(true)
      .withName("Drive_ResetOdometry");
  }

  /**
   * Reset the odometry to a known pose.
   */
  public Command resetOdometryCommand(Pose2d pose) {
    return runOnce(() -> resetOdometry(pose))
      .ignoringDisable(true)
      .withName("Drive_ResetOdometry");
  }

  /**
   * Enables/disables field-relative driving mode
   */
  public Command toggleFieldRelativeCommand() {
    return runOnce(() -> this.fieldRelative = !this.fieldRelative)
      .ignoringDisable(true)
      .withName("Drive_ToggleFieldRelativeMode");
  }

  /**
   * Enable/disable slow mode
   * @param slowMode True to enable slow mode, false to disable
   */
  public Command setSlowModeCommand(boolean slowMode) {
    return runOnce(() -> this.slowMode = slowMode)
      .ignoringDisable(true)
      .withName("Drive_SetSlowMode");
  }

  /**
   * Enable/disable motor brake mode
   */
  public Command setBrakeModeCommand(boolean brakeMode) {
    return runOnce(() -> setMotorBrake(brakeMode))
      .ignoringDisable(true)
      .withName("Drive_SetBrakeMode");
  }

  /**
   * Toggle motor brake mode
   */
  public Command toggleBrakeModeCommand() {
    return runOnce(() -> setMotorBrake(!brakeMode))
      .ignoringDisable(true)
      .withName("Drive_ToggleBrakeMode");
  }

  /**
   * Updates SmartDashboard with subsystem telemetry
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Field Relative", this::isFieldRelative, null);
    builder.addBooleanProperty("Slow Mode", this::isSlowMode, null);
    builder.addDoubleProperty("Accepted Vision Count", () -> Utils.showDouble(acceptedVisionCount), null);
    builder.addDoubleProperty("Rejected Vision Count", () -> Utils.showDouble(rejectedVisionCount), null);
    builder.addDoubleProperty("Blur Rejected Vision Count", () -> Utils.showDouble(blurRejectedVisionCount), null);
  }
}
