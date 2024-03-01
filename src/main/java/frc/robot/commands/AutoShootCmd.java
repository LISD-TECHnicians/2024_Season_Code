package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class AutoShootCmd extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  
  private final Timer shooterUpTimer = new Timer();
  private final Timer shooterDownTimer = new Timer();

  private double angle = PivotConstants.INTAKE_ANGLE;

  public AutoShootCmd(PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem, 
      ShooterSubsystem shooterSubsystem, LimelightSubsystem limelightSubsystem) {
    this.pivotSubsystem = pivotSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(shooterSubsystem, pivotSubsystem, indexerSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {
    limelightSubsystem.setPipeline(LimelightConstants.LL_TWO, LimelightConstants.AIM_PIPELINE);

    shooterUpTimer.restart();

    shooterDownTimer.reset();
    shooterDownTimer.stop();
  }

  @Override
  public void execute() {
    if (limelightSubsystem.getValidTag(LimelightConstants.LL_TWO) && (limelightSubsystem.getFiducialID(LimelightConstants.LL_TWO) == 7 
        || limelightSubsystem.getFiducialID(LimelightConstants.LL_TWO) == 4 || limelightSubsystem.getFiducialID(LimelightConstants.LL_TWO) == 8) 
        || limelightSubsystem.getFiducialID(LimelightConstants.LL_TWO) == 3) {
      angle = limelightSubsystem.getTY(LimelightConstants.LL_TWO) + PivotConstants.PIVOT_SHOOTER_OFFSET;
    }

    pivotSubsystem.setPivotAngle(angle);

    shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_DEFAULT_SPEED);

    if (pivotSubsystem.getShooterReadiness(angle) && shooterUpTimer.get() > ShooterConstants.SHOOTER_TIME_DELAY) {
      indexerSubsystem.setIndexerSpeed(IndexerConstants.INDEXER_DEFAULT_SPEED);

      shooterDownTimer.start();
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
    return shooterDownTimer.hasElapsed(1.50);
  }
}
