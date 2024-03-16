package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpTimeCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;

  private final Timer spinUpTimer = new Timer();

  public SpinUpTimeCmd(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    spinUpTimer.restart();
  }

  @Override
  public void execute() {
    shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_DEFAULT_SPEED);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return spinUpTimer.hasElapsed(ShooterConstants.SHOOTER_AUTO_SPIN_UP_DELAY);
  }
}
