package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoShootCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  
  private double angle;

  public AutoShootCmd(SwerveSubsystem swerveSubsystem, PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem, 
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(swerveSubsystem, shooterSubsystem, pivotSubsystem, indexerSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.setPipeline(LimelightConstants.LL_TWO, LimelightConstants.AIM_PIPELINE);
  }

  @Override
  public void execute() {
    angle = limelightSubsystem.getTY(LimelightConstants.LL_TWO) + PivotConstants.PIVOT_SHOOTER_OFFSET;

    pivotSubsystem.setPivotAngle(angle);
    swerveSubsystem.setSwerveRotation(limelightSubsystem.getTX(LimelightConstants.LL_TWO), DriveConstants.SWERVE_SHOOTER_OFFSET);

    shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_DEFAULT_SPEED);

    if (pivotSubsystem.getShooterReadiness(angle) && swerveSubsystem.getRotationReadiness(limelightSubsystem.getTX(LimelightConstants.LL_TWO), DriveConstants.SWERVE_SHOOTER_OFFSET)) {
      indexerSubsystem.setIndexerSpeed(IndexerConstants.INDEXER_DEFAULT_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.setIndexerSpeed(0.0);
    shooterSubsystem.setShooterSpeed(0.0);

    limelightSubsystem.setPipeline(LimelightConstants.LL_TWO, LimelightConstants.POSE_ESTIMATOR_PIPELINE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
