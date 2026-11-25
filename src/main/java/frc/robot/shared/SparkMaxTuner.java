// frc/robot/shared/SparkMaxTuner.java
package frc.robot.shared;

import java.util.Arrays;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.TuningConstants;

/**
 * Live closed-loop tuning from the dashboard for a SparkMax.
 * Only patches P/I/D and feedforward. Everything else (current limits, soft limits,
 * MAXMotion, idle mode) is left alone, and nothing is persisted to flash.
 */
public final class SparkMaxTuner {
  /** Initial gains, in the order they appear on the dashboard. */
  public record Gains(double p, double i, double d, double kS, double kV, double kA) {}

  private final String name;
  private final boolean tuneFlag;
  private final Gains base;
  private final SparkMax[] motors;
  private double[] last = new double[0];

  /**
   * @param name     Dashboard prefix, e.g. "Climber" -> "Climber/P"
   * @param tuneFlag The subsystem's TuningConstants flag
   * @param base     The gains already applied in the motor config
   * @param motor    The SparkMax to update
   */
  public SparkMaxTuner(String name, boolean tuneFlag, Gains base, SparkMax... motors) {
    this.name = name;
    this.tuneFlag = tuneFlag;
    this.base = base;
    this.motors = motors;

    if (tuneFlag) {
      SmartDashboard.putNumber(name + "/P", base.p());
      SmartDashboard.putNumber(name + "/I", base.i());
      SmartDashboard.putNumber(name + "/D", base.d());
      SmartDashboard.putNumber(name + "/kS", base.kS());
      SmartDashboard.putNumber(name + "/kV", base.kV());
      SmartDashboard.putNumber(name + "/kA", base.kA());

      // Seed from the published values so the first update() doesn't redundantly re-apply them
      last = read();
    }
  }

  /**
   * Reads the current values from the SmartDashboard, falling back to the base values if not present.
   * @return An array of the current gains [P, I, D, kS, kV, kA]
   */
  private double[] read() {
    return new double[] {
      SmartDashboard.getNumber(name + "/P", base.p()),
      SmartDashboard.getNumber(name + "/I", base.i()),
      SmartDashboard.getNumber(name + "/D", base.d()),
      SmartDashboard.getNumber(name + "/kS", base.kS()),
      SmartDashboard.getNumber(name + "/kV", base.kV()),
      SmartDashboard.getNumber(name + "/kA", base.kA())
    };
  }

  /** 
   * Call from the subsystem's periodic(). 
   */
  public void update() {
    if (!TuningConstants.isActive(tuneFlag)) return;

    double[] cur = read();
    if (Arrays.equals(cur, last)) return;
    last = cur;

    SparkMaxConfig tune = new SparkMaxConfig();
    tune.closedLoop
      .p(cur[0])
      .i(cur[1])
      .d(cur[2])
      .feedForward
        .kS(cur[3])
        .kV(cur[4])
        .kA(cur[5]);

    for (SparkMax motor : motors) {
      SparkMaxFactory.applyConfig(
        motor,
        tune,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters
      );
    }
  }
}
