package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDCmd extends Command {
  private final LEDSubsystem ledSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  public SetLEDCmd(LEDSubsystem ledSubsystem, IndexerSubsystem indexerSubsystem) {
    this.ledSubsystem = ledSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(ledSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!indexerSubsystem.getNotePresent()) {
      ledSubsystem.setColor((int)LEDConstants.GREEN.red, (int)LEDConstants.GREEN.green, (int)LEDConstants.GREEN.blue);
    }
    else if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        ledSubsystem.setColor((int)LEDConstants.RED.red, (int)LEDConstants.RED.green, (int)LEDConstants.RED.blue);
      }
      else {
        ledSubsystem.setColor((int)LEDConstants.BLUE.red, (int)LEDConstants.BLUE.green, (int)LEDConstants.BLUE.blue);
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
