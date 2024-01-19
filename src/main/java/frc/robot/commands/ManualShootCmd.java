package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShootCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private final DoubleSupplier desiredShooterSpeed;
  private final DoubleSupplier desiredPivotAngle;
  private final DoubleSupplier desiredIndexerSpeed;

  public ManualShootCmd(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, 
      DoubleSupplier desiredShooterSpeed, DoubleSupplier desiredPivotAngle, DoubleSupplier desiredIndexerSpeed) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    this.desiredShooterSpeed = desiredShooterSpeed;
    this.desiredPivotAngle = desiredPivotAngle;
    this.desiredIndexerSpeed = desiredIndexerSpeed;

    addRequirements(shooterSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.setPivotAngle(desiredPivotAngle.getAsDouble());
    shooterSubsystem.setShooterSpeed(desiredShooterSpeed.getAsDouble());

    if (Math.abs(desiredPivotAngle.getAsDouble() - shooterSubsystem.getPivotAngle()) < 1.0) {
      indexerSubsystem.setIndexerSpeed(desiredIndexerSpeed.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
