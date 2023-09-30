// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.TheGreatBalancingAct;
import frc.robot.commands.SwerveJoystickCommand.DodgeDirection;
// import frc.robot.commands.VisionAutos.ToNearestGridDebug;
import frc.robot.commands.autos.Auto3PieceLong;
import frc.robot.commands.autos.Auto3PieceLong;
import frc.robot.commands.autos.Auto3PieceShort;
import frc.robot.commands.autos.Balance;
import frc.robot.commands.autos.DirectBalance;
import frc.robot.commands.autos.PathPlannerAutos;
import frc.robot.commands.autos.SquareTest;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import frc.robot.subsystems.imu.Gyro;
import frc.robot.subsystems.imu.NavX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.swerve.SwerveDrivetrain.DRIVE_MODE;
import frc.robot.subsystems.swerve.SwerveDrivetrain.SwerveModuleType;
import frc.robot.subsystems.vision.primalWallnut.PrimalSunflower;

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
  public PowerDistribution pdp = new PowerDistribution(0, ModuleType.kCTRE);

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

  private PrimalSunflower backSunflower = new PrimalSunflower(VisionConstants.kLimelightBackName);
  private PrimalSunflower frontSunflower = new PrimalSunflower(VisionConstants.kLimelightFrontName);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    try {
      // Pass in "sunflowers" in reverse order of priority (most important last)
      swerveDrive = new SwerveDrivetrain(imu, SwerveModuleType.CANCODER, backSunflower, frontSunflower);
    } catch (IllegalArgumentException e) {
      DriverStation.reportError("Illegal Swerve Drive Module Type", e.getStackTrace());
    }

    initAutoChoosers();

    // Configure the trigger bindings
    configureBindings();

    DriverStation.reportWarning("Initalization complete", false);
  }

  public void initDefaultCommands() {
    wrist.resetEncoders(); // Wrist must start in stow position

    swerveDrive.setDefaultCommand(
      new SwerveJoystickCommand(
        swerveDrive,
        () -> -commandDriverController.getLeftY(), // Horizontal translation
        commandDriverController::getLeftX, // Vertical Translation
        // () -> 0.0, // debug
        commandDriverController::getRightX, // Rotationaq

        driverController::getTriangleButton, // Field oriented
        // () -> false, // Field oriented

        driverController::getCrossButton, // Towing
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
        () -> driverController.getR2Button(), // Precision mode (disabled)
        () -> driverController.getCircleButton(), // Turn to angle
        // () -> false, // Turn to angle (disabled)
        () -> { // Turn To angle Direction
          return 0.0;
        }
      ));

      wrist.setDefaultCommand(Commands.run(() -> wrist.moveWristMotionMagicJoystick(-operatorController.getLeftY()), wrist));
  }

  private void configureBindings() {
    // Note: whileTrue() does not restart the command if it ends while the button is
    // still being held
    // These button bindings are chosen for testing, and may be changed based on
    commandDriverController.share().onTrue(Commands.runOnce(imu::zeroHeading).andThen(() -> imu.setOffset(180)));
    commandDriverController.options().onTrue(Commands.runOnce(swerveDrive::resetEncoders));
    // commandDriverController.options().onTrue(Commands.runOnce(wrist::resetEncoders));
    // commandDriverController.PS().whileTrue(new TheGreatBalancingAct(swerveDrive));
    commandDriverController.triangle().onTrue(shooter.setPower(1)).onFalse(shooter.setPower(0));

    // commandDriverController.triangle().whileTrue(new TheGreatBalancingAct(swerveDrive));
    commandDriverController.circle()
      .onTrue(Commands.runOnce(() -> swerveDrive.setVelocityControl(true)))
      .onFalse(Commands.runOnce(() -> swerveDrive.setVelocityControl(false)));
    
    // Note:
    // L2:  hold = intake     let go = stow + hold
    // L1:  press = aim low   let go = score + stow
    // R1:  press = aim mid   let go = score + stow
    // R2:  press = aim high  let go = score + stow

    // commandOperatorController.square()
    //   .onTrue(temp += 10)
    //   .onFalse(temp -= 10);

    

    // Trigger cubeTrigger = new Trigger(shooter::hasCube);
    // cubeTrigger.onTrue(
    //   shooter.setPowerZero()
    //     .andThen(wrist.motionMagicCommand(WristConstants.kWristStow))
    //   );

    // Trigger voltTrigger = new Trigger(() -> Math.abs(pdp.getVoltage()) < 8);
    // voltTrigger.onTrue(
    //   Commands.sequence(
    //     Commands.waitSeconds(0.25),
    //     Commands.runOnce(() -> swerveDrive.resetEncoders())
    //   )
    // );

    // commandDriverController.L1()
    //   .whileTrue(Commands.sequence(
    //     // wrist.motionMagicCommand((WristConstants.kWristGround + WristConstants.kWristStow) / 2),
    //     // Commands.waitSeconds(0.25),
    //     wrist.motionMagicCommand((WristConstants.kWristStow))
    //     .andThen(shooter.holdUpdated()))
    //     .andThen(shooter.outtakeLowUpdated())
    //   )
    //   .onFalse(shooter.setPowerZero().alongWith(wrist.motionMagicCommand(WristConstants.kWristStow)));

    commandOperatorController.povUp()
      .onTrue(wrist.changeIntakeTargetTicks(100)
        .andThen(() -> SmartDashboard.putNumber("Intake Target Ticks", wrist.getIntakeTargetTicks())));

    
    commandOperatorController.povDown()
      .onTrue(wrist.changeIntakeTargetTicks(-100)
        .andThen(() -> SmartDashboard.putNumber("Intake Target Ticks", wrist.getIntakeTargetTicks())));

    
    commandOperatorController.options()
      .onTrue(wrist.resetIntakeTargetTicks()
        .andThen(() -> SmartDashboard.putNumber("Intake Target Ticks", wrist.getIntakeTargetTicks())));



    commandOperatorController.triangle()
    .onTrue(wrist.motionMagicCommand((WristConstants.kWristHigh))
      .andThen(shooter.outtakeHighUpdated()))
    .onFalse(wrist.motionMagicCommand((WristConstants.kWristStow))
      .andThen(shooter.setPowerZero()));

    commandOperatorController.circle()
    .onTrue(wrist.motionMagicCommand((WristConstants.kWristMid))
      .andThen(shooter.outtakeMidUpdated()))
    .onFalse(wrist.motionMagicCommand((WristConstants.kWristStow))
      .andThen(shooter.setPowerZero()));

    commandOperatorController.cross()
    .onTrue(wrist.motionMagicCommand((WristConstants.kWristLow))
      .andThen(shooter.outtakeLowUpdated()))
    .onFalse(wrist.motionMagicCommand((WristConstants.kWristStow))
      .andThen(shooter.setPowerZero()));

    commandOperatorController.L1()
      .onTrue(shooter.intakeUpdated()
        .andThen(wrist.motionMagicCommand(WristConstants.kWristSuperLowPickup)))
      .onFalse(shooter.holdUpdated());

    //Vaccuum / Mow the lawn
    commandOperatorController.L2() // Triangle
    .onTrue(shooter.intakeUpdated() // ** ADD TARGET TICKS CONSTANT FOR MOW
      .andThen(wrist.motionMagicCommand(wrist.getIntakeTargetTicks())
        .andThen(() -> SmartDashboard.putNumber("Intake Target Ticks", wrist.getIntakeTargetTicks()))))
    .onFalse(shooter.holdUpdated()
      .andThen(() -> wrist.setNormalTargetTicks()));

    commandOperatorController.R1()
      .onTrue(shooter.outtakeFullUpdated())
      .onFalse(shooter.setPowerZero()); 

    commandOperatorController.R2() // Circle
      .onTrue(wrist.motionMagicCommand(WristConstants.kWristStow))
      .onFalse(shooter.holdUpdated());

    


      


    // commandOperatorController.square()
    //   .onTrue(new InstantCommand(() -> new ToNearestGridDebug(swerveDrive, sunflower)));
    // commandOperatorController.cross()
    //   .onTrue(new InstantCommand(() -> sunflower.toNearestGrid()));
  }

  private void initAutoChoosers() {
    // Remember to load the pathplanner paths here
    final String[] paths = {
       "3Piece Short", "Balance", "DirectBalance", "3PieceLong2"
      // old paths "TestPath", "TestSquare", "Test Line", "TestSquare3", "SquareTest"
    };
    
    PathPlannerAutos.init(swerveDrive);

    for (String path : paths) {
      PathPlannerAutos.initPath(path);
      PathPlannerAutos.initPathGroup(path);
    }

    autoChooser.addOption("Do Nothing", Commands::none);
    autoChooser.addOption("PP Balance", () -> new Balance(PathPlannerAutos.autoBuilder, swerveDrive, shooter, wrist));
    autoChooser.addOption("PP 3Short", () -> new Auto3PieceShort(PathPlannerAutos.autoBuilder,swerveDrive, shooter, wrist));
    autoChooser.addOption("PP 3Long2", () -> new Auto3PieceLong(PathPlannerAutos.autoBuilder, swerveDrive, shooter, wrist));
    autoChooser.addOption("PP DirectBalance", () -> new DirectBalance(PathPlannerAutos.autoBuilder, swerveDrive, shooter, wrist));
    // autoChooser.addOption("PP SquareTest", () -> new SquareTest(PathPlannerAutos.autoBuilder));
    // these are the auto paths in the old format (not the actual full auto command)
    // autoChooser.addOption("Path Planner Test Auto", () -> PathPlannerAutos.pathplannerAuto("TestPath", swerveDrive));
    // autoChooser.addOption("Path Planner TestSquare", () -> PathPlannerAutos.pathplannerAuto("TestSquare", swerveDrive));
    // autoChooser.addOption("Path Planner Test3", () -> PathPlannerAutos.pathplannerAuto("Test Line", swerveDrive));
    // autoChooser.addOption("Path Planner TestSquare3", () -> PathPlannerAutos.pathplannerAuto("TestSquare3", swerveDrive));
    // autoChooser.addOption("Path Planner TestSquare4", () -> new SquareTest(PathPlannerAutos.autoBuilder));
    // autoChooser.addOption("Path Planner 3Piece Short", () -> PathPlannerAutos.pathplannerAuto("3Piece Short", swerveDrive));
    // autoChooser.addOption("Path Planner 3Piece Long", () -> PathPlannerAutos.pathplannerAuto("3Piece Long", swerveDrive));
    // autoChooser.addOption("Path Planner DirectBalance", () -> PathPlannerAutos.pathplannerAuto("DirectBalance", swerveDrive));

    ShuffleboardTab autosTab = Shuffleboard.getTab("Autos");

    autosTab.add("Selected Auto", autoChooser);
  }
  
  public void initShuffleboard() {
    imu.initShuffleboard(loggingLevel);
    wrist.initShuffleboard(loggingLevel);
    shooter.initShuffleboard(loggingLevel);
    // backSunflower.initShuffleboard(loggingLevel);
    // frontSunflower.initShuffleboard(loggingLevel);
    swerveDrive.initShuffleboard(loggingLevel);
    swerveDrive.initModuleShuffleboard(loggingLevel);
    ShuffleboardTab tab = Shuffleboard.getTab("Main");
    tab.addNumber("Total Current Draw", pdp::getTotalCurrent);
    tab.addNumber("Voltage", () -> Math.abs(pdp.getVoltage()));
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
