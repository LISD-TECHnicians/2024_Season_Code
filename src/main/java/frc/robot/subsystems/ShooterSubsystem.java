package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX pivotLeft = new TalonFX(ShooterConstants.PIVOT_LEFT_ID, "rio");
  // private final TalonFX pivotRight = new TalonFX(ShooterConstants.PIVOT_RIGHT_ID, "rio");

  private final CANSparkMax shooterLeft = new CANSparkMax(ShooterConstants.SHOOTER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(ShooterConstants.SHOOTER_RIGHT_ID, MotorType.kBrushless);

  public ShooterSubsystem() {
    shooterRight.follow(shooterLeft, true);
  }

  public void setShooterSpeed() {
    shooterLeft.set(1.0);
  }

  public void setShooterSpeed(double speed) {
    shooterLeft.set(speed);
  }

  public void setPivotAngle(double angle) {
    // INSERT PID
    
    pivotLeft.set(0);
  }

  public double getPivotAngle() {
    return pivotLeft.get(); //  convert to deg/s or rad/s
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
