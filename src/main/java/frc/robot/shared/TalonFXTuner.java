// frc/robot/shared/TalonFXTuner.java
package frc.robot.shared;

import java.util.Arrays;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.TuningConstants;

/**
 * Live Slot0 tuning from the dashboard for one or more TalonFXs.
 * Only touches Slot0, so current limits, soft limits, MotionMagic, etc. are left alone.
 */
public final class TalonFXTuner {
  private final String name;
  private final boolean tuneFlag;
  private final Slot0Configs base;
  private final boolean withGravity;
  private final TalonFX[] motors;
  private double[] last = new double[0];

  /**
   * @param name        Dashboard prefix, e.g. "Turret" -> "Turret/P"
   * @param tuneFlag    The subsystem's TuningConstants flag
   * @param base        The Slot0 you already put in the motor config (single source of truth)
   * @param withGravity True to expose kG (elevator/arm only)
   * @param motors      Every motor that should receive the new gains (leader and follower)
   */
  public TalonFXTuner(String name, boolean tuneFlag, Slot0Configs base, boolean withGravity, TalonFX... motors) {
    this.name = name;
    this.tuneFlag = tuneFlag;
    this.base = base;
    this.withGravity = withGravity;
    this.motors = motors;

    if (tuneFlag) {
      SmartDashboard.putNumber(name + "/P", base.kP);
      SmartDashboard.putNumber(name + "/I", base.kI);
      SmartDashboard.putNumber(name + "/D", base.kD);
      SmartDashboard.putNumber(name + "/kS", base.kS);
      SmartDashboard.putNumber(name + "/kV", base.kV);
      SmartDashboard.putNumber(name + "/kA", base.kA);
      if (withGravity) {
        SmartDashboard.putNumber(name + "/kG", base.kG);
      }

      // Seed from the published values so the first update() doesn't redundantly re-apply them
      last = read();
    }
  }

  /**
   * Reads the current values from the SmartDashboard, falling back to the base values if not present.
   * @return An array of the current tuning values: [kP, kI, kD, kS, kV, kA, kG]
   */
  private double[] read() {
    return new double[] {
      SmartDashboard.getNumber(name + "/P", base.kP),
      SmartDashboard.getNumber(name + "/I", base.kI),
      SmartDashboard.getNumber(name + "/D", base.kD),
      SmartDashboard.getNumber(name + "/kS", base.kS),
      SmartDashboard.getNumber(name + "/kV", base.kV),
      SmartDashboard.getNumber(name + "/kA", base.kA),
      withGravity ? SmartDashboard.getNumber(name + "/kG", base.kG) : base.kG
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

    Slot0Configs slot0 = new Slot0Configs()
      .withKP(cur[0])
      .withKI(cur[1])
      .withKD(cur[2])
      .withKS(cur[3])
      .withKV(cur[4])
      .withKA(cur[5])
      .withKG(cur[6])
      .withGravityType(base.GravityType);

    for (TalonFX motor : motors) {
      TalonFXFactory.applySlot0(motor, slot0);
    }
  }
}
