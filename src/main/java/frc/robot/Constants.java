package frc.robot;


public final class Constants {
  public static class ControllerConstants {
    public static final int CONTROLLER_1_PORT = 0;
    public static final int CONTROLLER_2_PORT = 1;
  }

  public static class DriveConstants {
    public static final double NOMINAL_VOLTAGE = 12.0;
  }

  public static class IntakeConstants {
    public static final int INTAKE_TOP_ID = 13;
    public static final int INTAKE_BOTTOM_ID = 14;
  }

  public static class IndexerConstants {
    public static final int INDEXER_LEFT_ID = 15;
    public static final int INDEXER_RIGHT_ID = 16;

    public static final int NOTE_PRESENT_PORT = 1;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_LEFT_ID = 17;
    public static final int SHOOTER_RIGHT_ID = 18;

    public static final int PIVOT_LEFT_ID = 19;
    public static final int PIVOT_RIGHT_ID = 20;

    public static final double PIVOT_P = 1.0;
    public static final double PIVOT_I = 1.0;
    public static final double PIVOT_D = 1.0;

    public static final double PIVOT_MAX_OUTPUT = 0.5;
  }

  public static class ClimberConstants {
    public static final int CLIMBER_LEFT_ID = 21;
    public static final int CLIMBER_RIGHT_ID = 22;

    public static final int LOWER_LIMIT_PORT = 0;
  }

  public static class LimelightConstants {
    public static final String LL_ONE = "limelight-llone";
    public static final String LL_TWO = "limelight-lltwo";
    
    public static final int POSE_ESTIMATOR_PIPELINE = 0;
    public static final int AIM_PIPELINE = 1;
    public static final int NOTE_DETECTION_PIPELINE = 2;
  }
}
