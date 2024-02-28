package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.LimelightHelpers;

import frc.robot.Constants.LimelightConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;

public class LimelightSubsystem extends SubsystemBase {
  private final DriverStation.Alliance alliance;

  public LimelightSubsystem() {
    this.alliance = DriverStation.getAlliance().get();

    // setPipeline(LimelightConstants.LL_ONE, LimelightConstants.NOTE_DETECTION_PIPELINE);
    setPipeline(LimelightConstants.LL_TWO, LimelightConstants.POSE_ESTIMATOR_PIPELINE);
  }

  public double getTX(String limelightName) {
    return LimelightHelpers.getTX(limelightName);
  }

  public double getTY(String limelightName) {
    return LimelightHelpers.getTY(limelightName);
  }

  public double getTA(String limelightName) {
    return LimelightHelpers.getTA(limelightName);
  }

  public double getNeuralID(String limelightName) {
    return LimelightHelpers.getNeuralClassID(limelightName);
  }
  
  public boolean getValidTag(String limelightName) {
    return LimelightHelpers.getTV(limelightName);
  }

  public Pose2d getPose(String limelightName) {
    if (alliance == DriverStation.Alliance.Blue) {
      return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
    }
    else {
      return LimelightHelpers.getBotPose2d_wpiRed(limelightName);
    }   

    // return LimelightHelpers.getBotPose2d(limelightName);
  }

  public double getFiducialID(String limelightName) {
    return LimelightHelpers.getFiducialID(limelightName);
  }

  public void setPipeline(String limelightName, int pipelineIndex) {
    LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
  }

  public double getPipeline(String limelightName) {
    return LimelightHelpers.getCurrentPipelineIndex(limelightName);
  }

  public double getTimeStamp(String limelightName) {
    return Timer.getFPGATimestamp() - (LimelightHelpers.getLatency_Capture(limelightName) + 
        LimelightHelpers.getLatency_Pipeline(limelightName)) / 1000;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Ready Aim", getValidTag(LimelightConstants.LL_TWO) && 
        (getFiducialID(LimelightConstants.LL_TWO) == 7 || getFiducialID(LimelightConstants.LL_TWO) == 4));
  }

  @Override
  public void simulationPeriodic() {}
}
