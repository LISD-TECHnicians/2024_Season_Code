package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterLeft = new CANSparkMax(ShooterConstants.SHOOTER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(ShooterConstants.SHOOTER_RIGHT_ID, MotorType.kBrushless);

  public ShooterSubsystem() {
    shooterLeft.restoreFactoryDefaults();
    shooterRight.restoreFactoryDefaults();

    shooterLeft.enableVoltageCompensation(ControllerConstants.NOMINAL_VOLTAGE);
    shooterRight.enableVoltageCompensation(ControllerConstants.NOMINAL_VOLTAGE);

    shooterLeft.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);
    shooterRight.setSmartCurrentLimit(ShooterConstants.SHOOTER_CURRENT_LIMIT);

    shooterLeft.setIdleMode(IdleMode.kCoast);
    shooterRight.setIdleMode(IdleMode.kCoast);

    shooterLeft.burnFlash();
    shooterRight.burnFlash();
  }

  public void setShooterSpeed(double speed) {
    shooterLeft.set(speed * ShooterConstants.SHOOTER_SPEED_FACTOR);
    shooterRight.set(-speed * ShooterConstants.SHOOTER_SPEED_FACTOR * ShooterConstants.SHOOTER_SPIN_FACTOR);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
