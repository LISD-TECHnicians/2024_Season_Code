package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.BooleanSupplier;

public class ManualClimberCmd extends Command {
  private final ClimberSubsystem climberSubsystem;

  private final BooleanSupplier up;
  private final BooleanSupplier down;

  public ManualClimberCmd(ClimberSubsystem climberSubsystem, BooleanSupplier up, BooleanSupplier down) {
    this.climberSubsystem = climberSubsystem;

    this.up = up;
    this.down = down;

    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (up.getAsBoolean()) {
      climberSubsystem.setClimberSpeed(ClimberConstants.CLIMBER_DEFAULT_SPEED);
    }
    else if (down.getAsBoolean()) {
      climberSubsystem.setClimberSpeed(-ClimberConstants.CLIMBER_DEFAULT_SPEED);
    }
    else {
      climberSubsystem.setClimberSpeed(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
