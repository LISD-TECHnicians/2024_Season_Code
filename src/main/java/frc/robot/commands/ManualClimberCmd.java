package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;

import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

public class ManualClimberCmd extends Command {
  private final ClimberSubsystem climberSubsystem;

  private final DoubleSupplier leftSpeed;
  private final DoubleSupplier rightSpeed;

  public ManualClimberCmd(ClimberSubsystem climberSubsystem, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
    this.climberSubsystem = climberSubsystem;

    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;

    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climberSubsystem.setClimberSpeed(Math.abs(leftSpeed.getAsDouble()) >= ControllerConstants.DEADBAND ? leftSpeed.getAsDouble() * ClimberConstants.CLIMBER_SPEED_FACTOR : 0.0,
      Math.abs(rightSpeed.getAsDouble()) >= ControllerConstants.DEADBAND ? rightSpeed.getAsDouble() * ClimberConstants.CLIMBER_SPEED_FACTOR : 0.0
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
