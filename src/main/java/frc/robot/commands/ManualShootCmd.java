package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ManualShootCmd extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  
  private final DoubleSupplier rawAngle;
  private final BooleanSupplier shoot;

  private double setAngle;

  public ManualShootCmd(PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem, 
      ShooterSubsystem shooterSubsystem, DoubleSupplier rawAngle, BooleanSupplier shoot) {
    this.pivotSubsystem = pivotSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    this.rawAngle = rawAngle;
    this.shoot = shoot;

    addRequirements(shooterSubsystem, pivotSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    setAngle = rawAngle.getAsDouble() /* Conversion */; 

    pivotSubsystem.setPivotAngle(setAngle);

    shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_DEFAULT_SPEED);

    if (shoot.getAsBoolean()) {
      indexerSubsystem.setIndexerSpeed(IndexerConstants.INDEXER_DEFAULT_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.setIndexerSpeed(0.0);
    shooterSubsystem.setShooterSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
