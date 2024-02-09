package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static class ControllerConstants {
    public static final int CONTROLLER_1_PORT = 0;
    public static final int CONTROLLER_2_PORT = 1;

    public static final double DEADBAND = 0.1; 
    public static final double LIMIT_VARIABILITY = 0.1;
    public static final double DEBOUNCE_TIME = 0.2;
  }

  public static class DriveConstants {
    public static final int FRONT_LEFT_DRIVE_ID = 8;
    public static final int FRONT_LEFT_ROTATION_ID = 12;
    public static final int FRONT_LEFT_ROTATION_ENCODER_ID = 4;
    public static final double FRONT_LEFT_ANGLE_OFFSET = 1.836;
    public static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERT = false;
    public static final boolean FRONT_LEFT_ROTATION_MOTOR_INVERT = true;
    public static final boolean FRONT_LEFT_ROTATION_ENCODER_INVERT = false;

    public static final int FRONT_RIGHT_DRIVE_ID = 7;
    public static final int FRONT_RIGHT_ROTATION_ID = 11;
    public static final int FRONT_RIGHT_ROTATION_ENCODER_ID = 3;
    public static final double FRONT_RIGHT_ANGLE_OFFSET = 2.029;
    public static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERT = true;
    public static final boolean FRONT_RIGHT_ROTATION_MOTOR_INVERT = true;
    public static final boolean FRONT_RIGHT_ROTATION_ENCODER_INVERT = false;

    public static final int REAR_RIGHT_DRIVE_ID = 6;
    public static final int REAR_RIGHT_ROTATION_ID = 10;
    public static final int REAR_RIGHT_ROTATION_ENCODER_ID = 2;
    public static final double REAR_RIGHT_ANGLE_OFFSET = 3.015; // In radians
    public static final boolean REAR_RIGHT_DRIVE_MOTOR_INVERT = true;
    public static final boolean REAR_RIGHT_ROTATION_MOTOR_INVERT = true;
    public static final boolean REAR_RIGHT_ROTATION_ENCODER_INVERT = false;

    public static final int REAR_LEFT_DRIVE_ID = 9;
    public static final int REAR_LEFT_ROTATION_ID = 13;
    public static final int REAR_LEFT_ROTATION_ENCODER_ID = 5;
    public static final double REAR_LEFT_ANGLE_OFFSET = -0.721;
    public static final boolean REAR_LEFT_DRIVE_MOTOR_INVERT = false;
    public static final boolean REAR_LEFT_ROTATION_MOTOR_INVERT = true;
    public static final boolean REAR_LEFT_ROTATION_ENCODER_INVERT = false;

    // Declare location of Swerve Modules relative to robot center 
    // X AXIS TOWARDS FRONT, Y AXIS TOWARDS LEFT
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.31, 0.31);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.31, -0.31);
    public static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(-0.31, -0.31);
    public static final Translation2d REAR_LEFT_LOCATION = new Translation2d(-0.31, 0.31);

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
      // Translation constants
      new PIDConstants(DriveConstants.PATH_TRANSLATION_P, DriveConstants.PATH_TRANSLATION_I, DriveConstants.PATH_TRANSLATION_D),  
      // Rotation constants
      new PIDConstants(DriveConstants.PATH_ROTATION_P, DriveConstants.PATH_ROTATION_I, DriveConstants.PATH_ROTATION_D),  
      DriveConstants.MAX_DRIVE_SPEED, 
      DriveConstants.SWERVE_RADIUS, // Drive base radius (distance from center to furthest module) 
      new ReplanningConfig()
    );

    public static final int PIGEON_ID = 14;

    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final double SWERVE_RADIUS = 0.44; // m

    public static final double WHEEL_DIAMETER = 0.102;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 1/6.75;

    public static final double DRIVE_MOTOR_POSITION_TO_METERS = DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
    public static final double DRIVE_MOTOR_VELOCITY_TO_METERS_SECOND = DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;

    public static final double MAX_DRIVE_SPEED = 4.97; // Max possible m/s
    public static final double MAX_DRIVE_SET_ACCELERATION = 10; // Max choosen m/s^2

    public static final double MAX_POSSIBLE_ROTATION_SPEED = MAX_DRIVE_SPEED / SWERVE_RADIUS;

    public static final double ROTATION_SPEED_SCALE_FACTOR = 0.75;

    public static final double MAX_SET_ROTATION_SPEED = MAX_POSSIBLE_ROTATION_SPEED * ROTATION_SPEED_SCALE_FACTOR;

    public static final double MOTOR_ROTATION_P = 0.1;
    public static final double MOTOR_ROTATION_I = 0.1; 
    public static final double MOTOR_ROTATION_D = 0.0;

    public static final double ROTATION_POSITION_CONTROL_P = 1.0; // Needs tuned
    public static final double ROTATION_POSITION_CONTROL_I = 5.0; 
    public static final double ROTATION_POSITION_CONTROL_D = 0.0;    

    public static final double PATH_TRANSLATION_P = 5.0;
    public static final double PATH_TRANSLATION_I = 0.0; 
    public static final double PATH_TRANSLATION_D = 0.0;

    public static final double PATH_ROTATION_P = 5.0;
    public static final double PATH_ROTATION_I = 0.0; 
    public static final double PATH_ROTATION_D = 0.0;

    public static final Pose2d ZERO_POSE = new Pose2d();
  }

  public static class IntakeConstants {
    public static final int INTAKE_ID = 13;

    public static final double INTAKE_SPEED_FACTOR = 0.75;
  }

  public static class IndexerConstants {
    public static final int INDEXER_LEFT_ID = 15;
    public static final int INDEXER_RIGHT_ID = 16;

    public static final int NOTE_PRESENT_PORT = 1;

    public static final double INDEXER_SPEED_FACTOR = 0.3;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_LEFT_ID = 17;
    public static final int SHOOTER_RIGHT_ID = 18;

    public static final double SHOOTER_RATE_LIMIT = 0.1;
    public static final double SHOOTER_SPEED_FACTOR = 0.8;
  }

  public static final class PivotConstants {
    public static final int PIVOT_LEFT_ID = 19;
    public static final int PIVOT_RIGHT_ID = 20;

    public static final double PIVOT_P = 1.0;
    public static final double PIVOT_I = 1.0;
    public static final double PIVOT_D = 1.0;

    public static final float PIVOT_FORWARD_LIMIT = 0;
    public static final float PIVOT_REVERSE_LIMIT = 0;

    public static final double PIVOT_MAX_OUTPUT = 0.5;

    public static final double INTAKE_ANGLE = 46;
    public static final double TRAVEL_ANGLE = 34;
    public static final double CLIMBER_ANGLE = 56;
  }

  public static class ClimberConstants {
    public static final int CLIMBER_LEFT_ID = 21;
    public static final int CLIMBER_RIGHT_ID = 22;

    public static final float CLIMBER_FORWARD_LIMIT = 0;
    public static final float CLIMBER_REVERSE_LIMIT = 0;

    public static final double CLIMBER_RATE_LIMIT = 3; 

    public static final double CLIMBER_SPEED_FACTOR = 0.8;
  }

  public static class LimelightConstants {
    public static final String LL_ONE = "limelight-llone";
    public static final String LL_TWO = "limelight-lltwo";
    
    public static final int POSE_ESTIMATOR_PIPELINE = 0;
    public static final int AIM_PIPELINE = 1;
    public static final int NOTE_DETECTION_PIPELINE = 2;
  }
}
