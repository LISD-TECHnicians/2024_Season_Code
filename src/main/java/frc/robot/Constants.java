package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {
  public static class ControllerConstants {
    public static final int CONTROLLER_1_PORT = 1;
    public static final int CONTROLLER_2_PORT = 0;

    public static final double DEADBAND = 0.15; 
    public static final double DEBOUNCE_TIME = 0.2;

    public static final double NOMINAL_VOLTAGE = 11.0;
  }

  public static class DriveConstants {
    public static final int FRONT_LEFT_DRIVE_ID = 9;
    public static final int FRONT_LEFT_ROTATION_ID = 12;
    public static final int FRONT_LEFT_ROTATION_ENCODER_ID = 5;
    public static final double FRONT_LEFT_ANGLE_OFFSET = 0.840;
    public static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERT = false;
    public static final boolean FRONT_LEFT_ROTATION_MOTOR_INVERT = true;
    public static final boolean FRONT_LEFT_ROTATION_ENCODER_INVERT = false;

    public static final int FRONT_RIGHT_DRIVE_ID = 6;
    public static final int FRONT_RIGHT_ROTATION_ID = 13;
    public static final int FRONT_RIGHT_ROTATION_ENCODER_ID = 2;
    public static final double FRONT_RIGHT_ANGLE_OFFSET = 1.459;
    public static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERT = true;
    public static final boolean FRONT_RIGHT_ROTATION_MOTOR_INVERT = true;
    public static final boolean FRONT_RIGHT_ROTATION_ENCODER_INVERT = false;

    public static final int REAR_RIGHT_DRIVE_ID = 8;
    public static final int REAR_RIGHT_ROTATION_ID = 10;
    public static final int REAR_RIGHT_ROTATION_ENCODER_ID = 4;
    public static final double REAR_RIGHT_ANGLE_OFFSET = 1.844; // In radians
    public static final boolean REAR_RIGHT_DRIVE_MOTOR_INVERT = true;
    public static final boolean REAR_RIGHT_ROTATION_MOTOR_INVERT = true;
    public static final boolean REAR_RIGHT_ROTATION_ENCODER_INVERT = false;

    public static final int REAR_LEFT_DRIVE_ID = 7;
    public static final int REAR_LEFT_ROTATION_ID = 11;
    public static final int REAR_LEFT_ROTATION_ENCODER_ID = 3;
    public static final double REAR_LEFT_ANGLE_OFFSET = 1.924;
    public static final boolean REAR_LEFT_DRIVE_MOTOR_INVERT = false;
    public static final boolean REAR_LEFT_ROTATION_MOTOR_INVERT = true;
    public static final boolean REAR_LEFT_ROTATION_ENCODER_INVERT = false;

    // Declare location of Swerve Modules relative to robot center 
    // X AXIS TOWARDS FRONT, Y AXIS TOWARDS LEFT
    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.314, 0.295); 
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.314, -0.295); 
    public static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(-0.111, -0.295); 
    public static final Translation2d REAR_LEFT_LOCATION = new Translation2d(-0.111, 0.295); 

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

    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int ROTATION_CURRENT_LIMIT = 40;

    public static final double SWERVE_RADIUS = 0.431; // m

    public static final double WHEEL_DIAMETER = 0.10070;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double DRIVE_GEAR_RATIO = 1/6.75;

    public static final double DRIVE_MOTOR_POSITION_TO_METERS = DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
    public static final double DRIVE_MOTOR_VELOCITY_TO_METERS_SECOND = DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;

    public static final double MAX_DRIVE_SPEED = 4.97; // Max possible m/s
    public static final double MAX_DRIVE_SET_ACCELERATION = 20; // Max choosen m/s^2

    public static final double MAX_POSSIBLE_ROTATION_SPEED = MAX_DRIVE_SPEED / SWERVE_RADIUS;

    public static final double ROTATION_SPEED_SCALE_FACTOR = 0.75;

    public static final double MAX_SET_ROTATION_SPEED = MAX_POSSIBLE_ROTATION_SPEED * ROTATION_SPEED_SCALE_FACTOR;

    public static final double MOTOR_ROTATION_P = 0.4;
    public static final double MOTOR_ROTATION_I = 0.3; 
    public static final double MOTOR_ROTATION_D = 0.0;

    public static final double CHASSIS_ROTATION_P = 0.2;
    public static final double CHASSIS_ROTATION_I = 0.0;
    public static final double CHASSIS_ROTATION_D = 0.0;

    public static final double PATH_TRANSLATION_P = 4.5;
    public static final double PATH_TRANSLATION_I = 0.0; 
    public static final double PATH_TRANSLATION_D = 0.0;

    public static final double PATH_ROTATION_P = 2.5;
    public static final double PATH_ROTATION_I = 0.0; 
    public static final double PATH_ROTATION_D = 0.0;

    public static final double DRIVE_VOLTAGE_DEADBAND = 1.5;

    public static final Pose2d ZERO_POSE = new Pose2d();

    public static final ChassisSpeeds INTAKE_REVERSE_SPEED = new ChassisSpeeds(-1.0, 0, 0);

    public static final double SWERVE_INTAKE_OFFSET = 2.0;
    public static final double SWERVE_SHOOTER_OFFSET = 2.0;

    public static final double SWERVE_ROTATION_VARIABILITY = 2.0;

    public static final double SLOW_SPEED_FACTOR = 0.45;
    public static final double SLOW_ROTATION_SPEED_FACTOR = 0.45;
  }

  public static class IntakeConstants {
    public static final int INTAKE_ID = 15;

    public static final int INTAKE_CURRENT_LIMIT = 70;

    public static final double INTAKE_SPEED_FACTOR = 0.70;
    public static final double INTAKE_DEFAULT_SPEED = 1.0;

    public static final double INTAKE_TIME_OUT = 7.00;
  }

  public static class IndexerConstants {
    public static final int INDEXER_LEFT_ID = 16;
    public static final int INDEXER_RIGHT_ID = 17;

    public static final int INDEXER_CURRENT_LIMIT = 35;

    public static final int NOTE_PRESENT_PORT = 0;

    public static final double INDEXER_SPEED_FACTOR = 0.25;
    public static final double INDEXER_DEFAULT_SPEED = 1.0;
  }

  public static final class PivotConstants {
    public static final int PIVOT_LEFT_ID = 18;
    public static final int PIVOT_RIGHT_ID = 19;

    public static final double PIVOT_P = 0.04;
    public static final double PIVOT_I = 0.0;
    public static final double PIVOT_D = 0.0;

    public static final float PIVOT_REVERSE_LIMIT = 0f; // Intake Position
    public static final float PIVOT_FORWARD_LIMIT = 9.31f;

    public static final int PIVOT_CURRENT_LIMIT = 25;

    public static final double PIVOT_MAX_OUTPUT = 0.2;

    public static final double PIVOT_GEAR_RATIO = 0.015238;

    public static final double PIVOT_INITIAL_ANGLE = 60.735;

    public static final double PIVOT_SHOOTER_OFFSET = 41.75;

    public static final double INTAKE_ANGLE = 56.50;
    public static final double TRAVEL_ANGLE = 31.478;
    public static final double CLIMBER_ANGLE = TRAVEL_ANGLE;
    public static final double SHOOT_ANGLE = 56.5;
    public static final double AMP_ANGLE = 60;

    public static final double PIVOT_VARIABILITY = 2.5;
  }

  public static class ShooterConstants {
    public static final int SHOOTER_LEFT_ID = 20;
    public static final int SHOOTER_RIGHT_ID = 21;

    public static final int SHOOTER_CURRENT_LIMIT = 25;

    public static final double SHOOTER_SPEED_FACTOR = 1.0;
    public static final double SHOOTER_DEFAULT_SPEED = 1.0;

    public static final double SHOOTER_AMP_SPEED = 0.15;

    public static final double SHOOTER_SPIN_FACTOR = 0.9;

    public static final double SHOOTER_TIME_UP_DELAY = 1.50;
    public static final double SHOOTER_TIME_DOWN_DELAY = 1.50;
    public static final double SHOOTER_AUTO_SPIN_UP_DELAY = 0.20;

    public static final double SHOOTER_TIME_OUT = 2.50;
  }

  public static class ClimberConstants {
    public static final int CLIMBER_LEFT_ID = 23;
    public static final int CLIMBER_RIGHT_ID = 22;

    public static final int CLIMBER_CURRENT_LIMIT = 80;

    public static final float CLIMBER_FORWARD_LIMIT = 123;
    public static final float CLIMBER_REVERSE_LIMIT = 0;

    public static final double CLIMBER_SPEED_FACTOR = 1.0;
    public static final double CLIMBER_DEFAULT_SPEED = 1.0;

    public static final double CLIMBER_VARIABILITY = 0.2;
  }

  public static class LimelightConstants {
    public static final String LL_ONE = "limelight-llone";
    public static final String LL_TWO = "limelight-lltwo";
    
    public static final int POSE_ESTIMATOR_PIPELINE = 0;
    public static final int AIM_PIPELINE = 1;

    public static final int NOTE_DETECTION_PIPELINE = 2;
    public static final int INTAKE_CAMERA = 3;

    public static final double LL_OVERRIDE_DEBOUNCE = 3;

    public static final double MIN_TA = 0.3;
    public static final double MAX_TX = 20;
  }

  public static final class LEDConstants {
    public static final int CANDLE_ID = 24;

    public static final double CANDLE_BRIGHTNESS = 1.0;

    public static final Color RED = new Color(255, 0, 0);
    public static final Color GREEN = new Color(0, 255, 0);
    public static final Color BLUE = new Color(0, 0, 255);
  }
}
