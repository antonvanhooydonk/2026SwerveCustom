package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.FieldConstants;
import frc.robot.shared.Utils;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

/**
 * Scoring command factory that provides a simple interface to trigger both
 * turret and shooter actions together. It can be used to give consistent 
 * behavior for scoring at the alliance hub.
 */
public class Scoring {
  private final DriveSubsystem driveSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;
  private final TurretSubsystem turretSubsystem;

  /**
   * Creates a scoring command factory that can coordinate the turret and shooter subsystems.
   * @param driveSubsystem the drive subsystem to control
   * @param feederSubsystem the feeder subsystem to control
   * @param flywheelSubsystem the flywheel subsystem to control
   * @param turretSubsystem the turret subsystem to control
   */
  public Scoring(
    DriveSubsystem driveSubsystem, 
    FeederSubsystem feederSubsystem, 
    FlywheelSubsystem flywheelSubsystem,
    TurretSubsystem turretSubsystem
  ) {
    this.driveSubsystem = driveSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;
    this.turretSubsystem = turretSubsystem;
  }

  private Command shootAt(Supplier<Pose2d> targetSupplier) {
    Supplier<Pose2d> turretPose = () -> turretSubsystem.getPose(driveSubsystem.getPose());
    Supplier<ChassisSpeeds> fieldSpeeds = driveSubsystem::getFieldRelativeSpeeds;

    // Rising-only debounce means this drops immediately when either condition fails
    Trigger ready = flywheelSubsystem.isFlywheelAtTargetTrigger
      .and(turretSubsystem.isAtTargetTrigger);

    return Commands.parallel(
      flywheelSubsystem.shootAtPoseCommand(turretPose, targetSupplier, fieldSpeeds),
      turretSubsystem.aimAtPoseCommand(turretPose, targetSupplier, fieldSpeeds),
      feederSubsystem.feedWhenReadyCommand(ready)
    );
  }

  public Command scoreCommand() {
    return shootAt(() -> Utils.isRedAlliance() ? FieldConstants.kRedHubPose : FieldConstants.kBlueHubPose);
  }

  public Command passCommand() {
    return shootAt(() -> Utils.isRedAlliance() ? FieldConstants.kRedPassPose : FieldConstants.kBluePassPose);
  }
}
