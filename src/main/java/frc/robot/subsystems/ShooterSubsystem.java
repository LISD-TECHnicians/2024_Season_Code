package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax shooterLeft = new CANSparkMax(ShooterConstants.SHOOTER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(ShooterConstants.SHOOTER_RIGHT_ID, MotorType.kBrushless);

  private final SlewRateLimiter shooterRateLimiter = new SlewRateLimiter(ShooterConstants.SHOOTER_RATE_LIMIT);

  public ShooterSubsystem() {
    shooterLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    shooterLeft.setIdleMode(IdleMode.kCoast);
    shooterRight.setIdleMode(IdleMode.kCoast);

    shooterRight.follow(shooterLeft, true);
  }

  public void setShooterSpeed(double speed) {
    shooterLeft.set(shooterRateLimiter.calculate(speed));
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
