package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PivotConstants;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;

import frc.robot.commands.AmpShootCmd;
import frc.robot.commands.AutoIntakeCmd;
import frc.robot.commands.AutoShootCmd;
import frc.robot.commands.ManualClimberCmd;
import frc.robot.commands.ManualIndexerCmd;
import frc.robot.commands.ManualIntakeCmd;
import frc.robot.commands.ManualReverseIntakeCmd;
import frc.robot.commands.ManualShootCmd;
import frc.robot.commands.SetPivotAngleCmd;
import frc.robot.commands.SpinUpCmd;
import frc.robot.commands.SwerveCmd;
import frc.robot.commands.VisionPoseUpdateCmd;


public class RobotContainer {
  private final CommandPS4Controller controller1 = new CommandPS4Controller(ControllerConstants.CONTROLLER_1_PORT);
  private final CommandPS4Controller controller2 = new CommandPS4Controller(ControllerConstants.CONTROLLER_2_PORT);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

  private final SwerveCmd joystickSwerve = new SwerveCmd(
      swerveSubsystem, 
      () -> Math.abs(controller1.getLeftY()) >= ControllerConstants.DEADBAND ? controller1.getLeftY() * DriveConstants.MAX_DRIVE_SPEED : 0.0, 
      () -> Math.abs(controller1.getLeftX()) >= ControllerConstants.DEADBAND ? controller1.getLeftX() * DriveConstants.MAX_DRIVE_SPEED : 0.0,
      () -> Math.abs(controller1.getRightX()) >= ControllerConstants.DEADBAND ? -controller1.getRightX() * DriveConstants.MAX_SET_ROTATION_SPEED : 0.0,
      controller1.L1(),
      controller1.L2());
  private final AutoShootCmd autoRunShooterCmd = new AutoShootCmd(/*swerveSubsystem, */pivotSubsystem, indexerSubsystem, shooterSubsystem, 
      limelightSubsystem);/*
  private final AutoIntakeCmd autoIntakeAlignCmd = new AutoIntakeCmd(swerveSubsystem, intakeSubsystem, pivotSubsystem, indexerSubsystem, 
      limelightSubsystem, true);*/
  private final ManualClimberCmd runClimberCmd = new ManualClimberCmd(climberSubsystem, () -> controller2.povUp().getAsBoolean(), () -> controller2.povDown().getAsBoolean());
  private final ManualIntakeCmd runIntakeCmd = new ManualIntakeCmd(intakeSubsystem, pivotSubsystem, indexerSubsystem);
  private final ManualShootCmd runShooterCmd = new ManualShootCmd(pivotSubsystem, indexerSubsystem, shooterSubsystem);
  private final SetPivotAngleCmd setPivotAngleCmd = new SetPivotAngleCmd(pivotSubsystem, PivotConstants.TRAVEL_ANGLE);
  private final ManualIndexerCmd runIndexerCmd = new ManualIndexerCmd(indexerSubsystem, controller2.button(4), controller2.button(2));
  private final ManualReverseIntakeCmd reverseIntakeCmd = new ManualReverseIntakeCmd(intakeSubsystem);
  private final AmpShootCmd ampShootCmd = new AmpShootCmd(intakeSubsystem, pivotSubsystem, indexerSubsystem, shooterSubsystem);
  private final SpinUpCmd spinUpCmd = new SpinUpCmd(shooterSubsystem, controller2.button(3));

  private final VisionPoseUpdateCmd visionPoseUpdateCmd = new VisionPoseUpdateCmd(swerveSubsystem, limelightSubsystem);

  private final Trigger poseUpdate = new Trigger(() -> (
      limelightSubsystem.getValidTag(LimelightConstants.LL_TWO) && 
      (Math.abs(limelightSubsystem.getTA(LimelightConstants.LL_TWO)) > LimelightConstants.MIN_TA) &&
      (Math.abs(limelightSubsystem.getTX(LimelightConstants.LL_TWO)) < LimelightConstants.MAX_TX) && 
      limelightSubsystem.getPipeline(LimelightConstants.LL_TWO) == LimelightConstants.POSE_ESTIMATOR_PIPELINE)
  );

  public static ShuffleboardTab robotStatus = Shuffleboard.getTab("Robot");

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    /*NamedCommands.registerCommand("Auto Shoot", new AutoShootCmd(pivotSubsystem, indexerSubsystem, shooterSubsystem, 
        limelightSubsystem).asProxy());
    NamedCommands.registerCommand("Intake", new ManualIntakeCmd(intakeSubsystem, pivotSubsystem, indexerSubsystem).asProxy());
    NamedCommands.registerCommand("Auto Align Intake", new AutoIntakeCmd(swerveSubsystem, intakeSubsystem, pivotSubsystem, 
        indexerSubsystem, limelightSubsystem, true));
    NamedCommands.registerCommand("Auto No Align Intake", new AutoIntakeCmd(swerveSubsystem, intakeSubsystem, pivotSubsystem, 
        indexerSubsystem, limelightSubsystem, false)); */
    NamedCommands.registerCommand("Manual Shoot", new ManualShootCmd(pivotSubsystem, indexerSubsystem, shooterSubsystem));
    NamedCommands.registerCommand("Manual Intake", new ManualIntakeCmd(intakeSubsystem, pivotSubsystem, indexerSubsystem));

    configureBindings();

    swerveSubsystem.setDefaultCommand(joystickSwerve);
    climberSubsystem.setDefaultCommand(runClimberCmd);
    pivotSubsystem.setDefaultCommand(setPivotAngleCmd);
    indexerSubsystem.setDefaultCommand(runIndexerCmd);  
    shooterSubsystem.setDefaultCommand(spinUpCmd);  

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    // Driver Controls
    controller1.R1().debounce(ControllerConstants.DEBOUNCE_TIME).whileTrue(runIntakeCmd);
    controller1.button(3).debounce(ControllerConstants.DEBOUNCE_TIME).whileTrue(reverseIntakeCmd);
    // controller1.button(1).debounce(ControllerConstants.DEBOUNCE_TIME).whileTrue(autoIntakeAlignCmd);

    // Operator Controls
    controller2.R2().debounce(ControllerConstants.DEBOUNCE_TIME).whileTrue(runShooterCmd);
    controller2.L2().debounce(ControllerConstants.DEBOUNCE_TIME).whileTrue(autoRunShooterCmd);

    controller2.povUp().or(controller2.povDown()).debounce(ControllerConstants.DEBOUNCE_TIME).whileTrue(runClimberCmd);

    // controller2.L1().debounce(ControllerConstants.DEBOUNCE_TIME).whileTrue(ampShootCmd);

    poseUpdate.whileTrue(visionPoseUpdateCmd);
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
