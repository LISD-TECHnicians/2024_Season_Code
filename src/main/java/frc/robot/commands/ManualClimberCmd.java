package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ClimberConstants;

import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

public class ManualClimberCmd extends Command {
  private final ClimberSubsystem climberSubsystem;

  private final DoubleSupplier speed;

  public ManualClimberCmd(ClimberSubsystem climberSubsystem, DoubleSupplier speed) {
    this.climberSubsystem = climberSubsystem;

    this.speed = speed;

    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climberSubsystem.setClimberSpeed(speed.getAsDouble() >= 0 ? speed.getAsDouble() : speed.getAsDouble() * 
        ClimberConstants.CLIMBER_DOWN_SPEED_FACTOR);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
