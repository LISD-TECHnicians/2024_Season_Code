package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.wpilibj.DigitalInput;

public class IndexerSubsystem extends SubsystemBase {
  private final CANSparkMax indexerLeft = new CANSparkMax(IndexerConstants.INDEXER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax indexerRight = new CANSparkMax(IndexerConstants.INDEXER_RIGHT_ID, MotorType.kBrushless);

  private final DigitalInput notePresent = new DigitalInput(IndexerConstants.NOTE_PRESENT_PORT);

  private final SlewRateLimiter indexerRateLimiter = new SlewRateLimiter(IndexerConstants.INDEXER_RATE_LIMIT);

  public IndexerSubsystem() {
    indexerLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    indexerRight.follow(indexerLeft, true);
  }

  public void setIndexerSpeed() {
    indexerLeft.set(indexerRateLimiter.calculate(1.0));
  }

  public void setIndexerSpeed(double speed) {
    indexerLeft.set(indexerRateLimiter.calculate(speed));
  }

  public boolean getNotePresent() {
    return notePresent.get();
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
