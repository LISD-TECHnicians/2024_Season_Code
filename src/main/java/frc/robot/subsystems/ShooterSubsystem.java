package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax pivotLeft = new CANSparkMax(ShooterConstants.PIVOT_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax pivotRight = new CANSparkMax(ShooterConstants.PIVOT_RIGHT_ID, MotorType.kBrushless);

  private final CANSparkMax shooterLeft = new CANSparkMax(ShooterConstants.SHOOTER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax shooterRight = new CANSparkMax(ShooterConstants.SHOOTER_RIGHT_ID, MotorType.kBrushless);

  private final PIDController pivotPID = new PIDController(ShooterConstants.PIVOT_P, ShooterConstants.PIVOT_I, ShooterConstants.PIVOT_D);

  // Encoder in Right SparkMax, Limits in Left SparkMax

  public ShooterSubsystem() {
    pivotLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);
    shooterLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    pivotLeft.setIdleMode(IdleMode.kBrake);
    pivotRight.setIdleMode(IdleMode.kBrake);

    shooterLeft.setIdleMode(IdleMode.kCoast);
    shooterRight.setIdleMode(IdleMode.kCoast);

    pivotLeft.setSoftLimit(SoftLimitDirection.kForward, ShooterConstants.PIVOT_FORWARD_LIMIT);
    pivotLeft.setSoftLimit(SoftLimitDirection.kReverse, ShooterConstants.PIVOT_REVERSE_LIMIT);

    pivotLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    pivotLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

    pivotRight.follow(pivotLeft, true);
    shooterRight.follow(shooterLeft, true);
  }

  public void setShooterSpeed(double speed) {
    shooterLeft.set(speed);
  }

  public void setPivotAngle(double angle) {
    pivotLeft.set(MathUtil.clamp(pivotPID.calculate(getPivotAngle(), angle), -ShooterConstants.PIVOT_MAX_OUTPUT, ShooterConstants.PIVOT_MAX_OUTPUT));
  }

  public double getPivotAngle() {
    return pivotRight.getEncoder().getPosition(); //  convert to radians
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
