package frc.robot.subsystems.superstructure;

public class SuperstructureConstants {

  public static class IDs {
    // -- INTAKE
    public static final int INTAKE_PIVOT_ID = 16;
    public static final int INTAKE_ROLLER_ID = 17;

    // -- SHOOTER
    public static final int SHOOTER_HOOD_ID = 31;
    public static final int SHOOTER_ROLLERS_ID = 30;

    // -- TRANSFER
    public static final int TRANSFER_ID = 22;
  }

  public static class IntakeConstants {
    // -- PIVOT CONSTANTS
    public static final double IN = 0.0;
    public static final double OUT = 0.0;

    // -- ROLLER CONSTANTS
    public static final double ZERO_RVOLTAGE = 0.0;
    public static final double FORWARD_RVOLTAGE = 8.0;
    public static final double REVERSE_RVOLTAGE = 8.0;

    public static final double NONE_RPS = 0.0;
    public static final double FORWARD_RPS = 25.0;
    public static final double REVERSE_RPS = 20.0;
  }

  public static class ShooterConstants {
    public static final double ROLLERS_RPS = 50.0;
  }

  public static class TransferConstants {
    public static final double FORWARD_RPS = 30.0;
    public static final double REVERSE_RPS = 15.0;
  }
}
