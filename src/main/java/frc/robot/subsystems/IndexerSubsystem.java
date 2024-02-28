package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IndexerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkMax indexerLeft = new CANSparkMax(IndexerConstants.INDEXER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax indexerRight = new CANSparkMax(IndexerConstants.INDEXER_RIGHT_ID, MotorType.kBrushless);

  private final DigitalInput notePresent = new DigitalInput(IndexerConstants.NOTE_PRESENT_PORT);

  public IndexerSubsystem() {
    indexerLeft.restoreFactoryDefaults();
    indexerRight.restoreFactoryDefaults();

    indexerLeft.enableVoltageCompensation(ControllerConstants.NOMINAL_VOLTAGE);
    indexerRight.enableVoltageCompensation(ControllerConstants.NOMINAL_VOLTAGE);

    indexerLeft.setSmartCurrentLimit(IndexerConstants.INDEXER_CURRENT_LIMIT);
    indexerRight.setSmartCurrentLimit(IndexerConstants.INDEXER_CURRENT_LIMIT);

    indexerLeft.setIdleMode(IdleMode.kBrake);
    indexerRight.setIdleMode(IdleMode.kBrake);

    indexerRight.follow(indexerLeft, true);

    indexerLeft.burnFlash();
    indexerRight.burnFlash();
  }

  public void setIndexerSpeed(double speed) {
    indexerLeft.set(speed * IndexerConstants.INDEXER_SPEED_FACTOR);
  }

  public boolean getNotePresent() {
    return notePresent.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Note Present", !getNotePresent());
  }

  @Override
  public void simulationPeriodic() {}
}
