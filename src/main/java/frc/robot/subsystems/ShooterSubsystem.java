package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax pivotLeft = new CANSparkMax(ShooterConstants.PIVOT_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax pivotRight = new CANSparkMax(ShooterConstants.PIVOT_RIGHT_ID, MotorType.kBrushless);

  private final CANSparkMax shooterLeft = new CANSparkMax(ShooterConstants.SHOOTER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(ShooterConstants.SHOOTER_RIGHT_ID, MotorType.kBrushless);

  private final PIDController pivotPID = new PIDController(ShooterConstants.PIVOT_P, ShooterConstants.PIVOT_I, ShooterConstants.PIVOT_D);

  public ShooterSubsystem() {
    pivotLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);
    shooterLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    pivotRight.follow(pivotLeft, true);
    shooterRight.follow(shooterLeft, true);
  }

  public void setShooterSpeed() {
    shooterLeft.set(1.0);
  }

  public void setShooterSpeed(double speed) {
    shooterLeft.set(speed);
  }

  public void setPivotAngle(double angle) {
    pivotLeft.set(MathUtil.clamp(pivotPID.calculate(getPivotAngle(), angle), -ShooterConstants.PIVOT_MAX_OUTPUT, ShooterConstants.PIVOT_MAX_OUTPUT));
  }

  public double getPivotAngle() {
    return pivotLeft.getEncoder().getPosition(); //  convert to radians
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
