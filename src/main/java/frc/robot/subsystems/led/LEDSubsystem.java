// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PWMConstants;
import frc.robot.util.Utils;

/**
 * LED subsystem for controlling an addressable LED strip and providing visual feedback. 
 */
public class LEDSubsystem extends SubsystemBase {
  // LED hardware
  private final AddressableLED ledStrip;
  private final AddressableLEDBuffer ledBuffer;
      
  /**
   * Creates a new LEDSubsystem.
   */
  public LEDSubsystem() {
    // Initialize LED strip
    ledBuffer = new AddressableLEDBuffer(LEDConstants.kLEDCount);
    ledStrip = new AddressableLED(PWMConstants.kLEDStringID);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();

    // Set subsystem default command
    setDefaultCommand(offCommand());
    
    // Output initialization progress
    Utils.logInfo("LED subsystem initialized");
  }

  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);
  }

  // ----------------------------------------------------------------------------------------
  // Private state methods
  // ----------------------------------------------------------------------------------------
  
  /**
   * Applies the specified LED pattern to the LED strip.
   * The pattern's brightness is adjusted dimmed for
   * photo-sensitivity reasons prior to display.
   * @param pattern The LED pattern to apply
   */
  private void setPattern(LEDPattern pattern) {
    pattern.atBrightness(LEDConstants.kBrightness).applyTo(ledBuffer);
  }

  /**
   * Cancels the currently running command on the LED subsystem, if any.
   * The default command will take over after cancellation.
   */
  private void cancelCurrent() {
    Command current = getCurrentCommand();
    if (current != null) {
      current.cancel();
    }
  }

  // ----------------------------------------------------------------------------------------
  // Public methods to run at different phases of the match
  // ----------------------------------------------------------------------------------------

  /**
   * Initializes the LED subsystem at the start of the autonomous phase.
   */
  public void autonomousInit() {
    cancelCurrent();
    Utils.logInfo("LED subsystem initialized for autonomous");
  }

  /**
   * Initializes the LED subsystem at the start of the teleop phase.
   */
  public void teleopInit() {
    cancelCurrent();
    Utils.logInfo("LED subsystem initialized for teleop");
  }

  /**
   * Initializes the LED subsystem for post match (disabled) state.
   */
  public void postMatch() {
    cancelCurrent();
    Utils.logInfo("LED subsystem initialized for post match");
  }

  // ----------------------------------------------------------------------------------------
  // Public Command Factory Methods
  // ----------------------------------------------------------------------------------------
  
  /**
   * Applies simple animated or solid patterns to the LED strip.
   * Append .withTimeout() to the returned command to have it run for a specific duration.
   * @param pattern The LED pattern to apply
   * @return Command that applies the specified LED pattern
   */
  public Command setPatternCommand(LEDPattern pattern) {
    return run(() -> setPattern(pattern))
      .ignoringDisable(true)
      .withName("LED_Custom");
  }

  /**
   * Command to set the LED strip off until another command takes over
   * @return Command that sets LED display off
   */
  public Command offCommand() {
    return setPatternCommand(LEDPattern.solid(Color.kBlack))
      .withName("LED_Off");
  }
  
  /**
   * Command to briefly indicate success state
   * @return Command with success LED
   */
  public Command successCommand() {
    return setPatternCommand(LEDPattern.solid(Color.kLime))
      .withTimeout(1.0)
      .withName("LED_Success");
  }
  
  /**
   * Command to briefly indicate info state
   * @return Command with info LED
   */
  public Command infoCommand() {
    return setPatternCommand(LEDPattern.solid(Color.kCyan))
      .withTimeout(1.0)
      .withName("LED_Info");
  }
  
  /**
   * Command to briefly indicate warning state
   * @return Command with warning LED
   */
  public Command warningCommand() {
    return setPatternCommand(LEDPattern.solid(Color.kGold))
      .withTimeout(1.0)
      .withName("LED_Warning");
  }
  
  /**
   * Command to indicate error state until interrupted
   * @return Command with error LED
   */
  public Command errorCommand() {
    return setPatternCommand(LEDPattern.solid(Color.kMagenta))
      .withName("LED_Error");
  }
}
