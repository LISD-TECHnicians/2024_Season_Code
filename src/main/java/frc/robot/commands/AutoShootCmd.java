package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoShootCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  // Also need subsystem to rotate bot

  public AutoShootCmd(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, LimelightSubsystem limelightSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(shooterSubsystem, indexerSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double pivotAngle = limelightSubsystem.getTY(LimelightConstants.LL_ONE) + 10;

    shooterSubsystem.setPivotAngle(pivotAngle);
    shooterSubsystem.setShooterSpeed(1);

    if (Math.abs(pivotAngle - shooterSubsystem.getPivotAngle()) < 1.0) {
      indexerSubsystem.setIndexerSpeed(1);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
