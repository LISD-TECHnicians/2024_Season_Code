package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkMax indexerLeft = new CANSparkMax(IndexerConstants.INDEXER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax indexerRight = new CANSparkMax(IndexerConstants.INDEXER_RIGHT_ID, MotorType.kBrushless);

  private final DigitalInput notePresent = new DigitalInput(IndexerConstants.NOTE_PRESENT_PORT);

  public IndexerSubsystem() {
    indexerLeft.restoreFactoryDefaults();
    indexerRight.restoreFactoryDefaults();

    indexerLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);
    indexerRight.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    indexerLeft.setSmartCurrentLimit(ControllerConstants.DEFAULT_550_CURRENT_LIMIT);
    indexerRight.setSmartCurrentLimit(ControllerConstants.DEFAULT_550_CURRENT_LIMIT);

    indexerLeft.setIdleMode(IdleMode.kBrake);
    indexerRight.setIdleMode(IdleMode.kBrake);

    indexerRight.follow(indexerLeft, true);
  }

  public void setIndexerSpeed(double speed) {
    indexerLeft.set(speed * IndexerConstants.INDEXER_SPEED_FACTOR);
  }

  public boolean getNotePresent() {
    return notePresent.get();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
