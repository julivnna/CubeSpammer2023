// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.PathPlannerAutos;
import frc.robot.commands.SquareTest;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TheGreatBalancingAct;
import frc.robot.commands.SwerveJoystickCommand.DodgeDirection;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.NavX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.swerve.SwerveDrivetrain.SwerveModuleType;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public Wrist wrist = new Wrist();
  public Gyro imu = new NavX();
  // public Gyro imu = new Pigeon(60);
  public SwerveDrivetrain swerveDrive;
  public Shooter shooter = new Shooter();

  private final CommandPS4Controller commandDriverController = new CommandPS4Controller(
      ControllerConstants.kDriverControllerPort);
  private final PS4Controller driverController = commandDriverController.getHID();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandPS4Controller commandOperatorController = new CommandPS4Controller(
      ControllerConstants.kOperatorControllerPort);
  private final PS4Controller operatorController = commandOperatorController.getHID();
  // private final Joystick joystick = new Joystick(2);

  private final LOG_LEVEL loggingLevel = LOG_LEVEL.ALL;
  private final POVButton upButton = new POVButton (operatorController,0);
  private final POVButton rightButton = new POVButton (operatorController, 90);
  private final POVButton downButton = new POVButton (operatorController, 180);
  private final POVButton leftButton = new POVButton (operatorController, 270);

  private final POVButton upButtonDriver = new POVButton (driverController, 0);
  private final POVButton rightButtonDriver = new POVButton (driverController, 90);
  private final POVButton downButtonDriver = new POVButton (driverController, 180);
  private final POVButton leftButtonDriver = new POVButton (driverController, 270);

  private SendableChooser<Supplier<CommandBase>> autoChooser = new SendableChooser<Supplier<CommandBase>>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.CANCODER);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }

    initAutoChoosers();

    // Configure the trigger bindings
    configureBindings();

    DriverStation.reportWarning("Initalization complete", false);
  }

  public void initDefaultCommands() {
    swerveDrive.setDefaultCommand(
      new SwerveJoystickCommand(
        swerveDrive,
        () -> -commandDriverController.getLeftY(), // Horizontal translation
        commandDriverController::getLeftX, // Vertical Translation
        // () -> 0.0, // debug
        commandDriverController::getRightX, // Rotation
        // driverController::getSquareButton, // Field oriented
        () -> false, // Field oriented
        driverController::getSquareButton, // Towing
        // Dodge
        // () -> {return badPS5.getL1Button() || badPS5.getR1Button();},
        () -> false,
        // Dodging
        () -> {
          // if (badPS5.getL1Button()) {
          //   return DodgeDirection.LEFT;
          // } 
          // if (badPS5.getR1Button()) {
          //   return DodgeDirection.RIGHT;
          // }
          return DodgeDirection.NONE;
        },
        // driverController::getR2Button, // Precision/"Sniper Button"
        () -> false, // Precision mode (disabled)
        // () -> driverController.getR1Button() || driverController.getL1Button(), // Turn to angle
        () -> false, // Turn to angle (disabled)
        () -> { // Turn To angle Direction
          if (driverController.getR1Button()) {
            return 180.0;
          } else {
            return 0.0;
          }
        }
      ));
  }

  private void configureBindings() {
    // Note: whileTrue() does not restart the command if it ends while the button is
    // still being held
    // These button bindings are chosen for testing, and may be changed based on
    commandDriverController.share().onTrue(Commands.runOnce(imu::zeroHeading));
    commandDriverController.options().onTrue(Commands.runOnce(swerveDrive::resetEncoders));
    commandDriverController.options().onTrue(Commands.runOnce(wrist::resetEncoders));

    commandDriverController.triangle().whileTrue(new TheGreatBalancingAct(swerveDrive));
    commandDriverController.circle()
      .onTrue(Commands.runOnce(() -> swerveDrive.setVelocityControl(true)))
      .onFalse(Commands.runOnce(() -> swerveDrive.setVelocityControl(false)));
    
    // Note:
    // L2:  hold = intake     let go = stow + hold
    // L1:  press = aim low   let go = score + stow
    // R1:  press = aim mid   let go = score + stow
    // R2:  press = aim high  let go = score + stow
    commandDriverController.L2()
      .onTrue(wrist.motionMagicCommand((WristConstants.kWristGround))
        .andThen(shooter.setPower(ShooterConstants.kIntakePower)))
      .onFalse(wrist.motionMagicCommand(WristConstants.kWristStow)
        .andThen(shooter.setPower(ShooterConstants.kIntakeNeutralPower)));

    Trigger cubeTrigger = new Trigger(shooter::hasCube);
    cubeTrigger.onTrue(
      shooter.setPowerZero()
        .andThen(wrist.motionMagicCommand(WristConstants.kWristStow))
      );

    commandDriverController.L1()
      .onTrue(wrist.motionMagicCommand((WristConstants.kWristLow)))
      .onFalse(
        shooter.outtakeLow()
        .andThen(wrist.motionMagicCommand(WristConstants.kWristStow))
      );
    
    commandDriverController.R1()
      .onTrue(wrist.motionMagicCommand((WristConstants.kWristMid)))
      .onFalse(
        shooter.outtakeMid()
        .andThen(wrist.motionMagicCommand(WristConstants.kWristStow))
      );

    commandDriverController.R2()
      .onTrue(wrist.motionMagicCommand((WristConstants.kWristHigh)))
      .onFalse(
        shooter.outtakeHigh()
        .andThen(wrist.motionMagicCommand(WristConstants.kWristStow))
      );

  }

  private void initAutoChoosers() {
    // Remember to load the pathplanner paths here
    final String[] paths = {
      "TestPath", "TestSquare", "Test Line", "TestSquare3"
    };
    
    PathPlannerAutos.init(swerveDrive);

    for (String path : paths) {
      PathPlannerAutos.initPath(path);
      PathPlannerAutos.initPathGroup(path);
    }

    autoChooser.addOption("Do Nothing", Commands::none);
    autoChooser.addOption("Path Planner Test Auto", () -> PathPlannerAutos.pathplannerAuto("TestPath", swerveDrive));
    autoChooser.addOption("Path Planner TestSquare", () -> PathPlannerAutos.pathplannerAuto("TestSquare", swerveDrive));
    autoChooser.addOption("Path Planner Test3", () -> PathPlannerAutos.pathplannerAuto("Test Line", swerveDrive));
    autoChooser.addOption("Path Planner TestSquare3", () -> PathPlannerAutos.pathplannerAuto("TestSquare3", swerveDrive));
    autoChooser.addOption("Path Planner TestSquare4", () -> new SquareTest(PathPlannerAutos.autoBuilder));

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    wrist.initShuffleboard(loggingLevel);
    shooter.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
  }

  public void reportAllToSmartDashboard() {
    imu.reportToSmartDashboard(loggingLevel);
    wrist.reportToSmartDashboard(loggingLevel);
    shooter.reportToSmartDashboard(loggingLevel);
    swerveDrive.reportToSmartDashboard(loggingLevel);
    swerveDrive.reportModulesToSmartDashboard(loggingLevel);
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command currentAuto = autoChooser.getSelected().get();

    swerveDrive.setDriveMode(DRIVE_MODE.AUTONOMOUS);
    return currentAuto;
  }
}
