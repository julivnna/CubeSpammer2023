package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Reportable.LOG_LEVEL;
import static frc.robot.Constants.*;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.FeedbackConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfigurator;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

/**
 * Swerve module that uses CANCoder for the absolute position
 */
public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder canCoder;

    private final DutyCycleOut stopRequest;
    private final DutyCycleOut driveRequest; 
    private final PositionDutyCycle turnPositionRequest;
    private final VelocityDutyCycle driveVelocityRequest;

    private final Slot0Configs drivePIDConfigs;
    private final Slot0Configs turnPIDConfigs;

    private final int driveMotorID;
    private final int turnMotorID;
    private final int CANCoderID;

    private double currentPercent = 0;
    private double currentAngle = 0;
    private double desiredRotations = 0;
    private double desiredVelocity = 0;
    private boolean velocityControl = false;

    private SwerveModuleState desiredState = null;
    private SwerveModulePosition currPosition = new SwerveModulePosition();
    private SwerveModuleState currState = new SwerveModuleState();

    /**
     * Construct a new CANCoder Swerve Module.
     * 
     * @param driveMotorId
     * @param turningMotorId
     * @param invertDriveMotor
     * @param invertTurningMotor
     * @param CANCoderId
     */
    public SwerveModule(int driveMotorId, int turningMotorId, boolean invertDriveMotor, 
                        boolean invertTurningMotor, int CANCoderId) {
        this.driveMotor = new TalonFX(driveMotorId, ModuleConstants.kCANivoreName);
        this.turnMotor = new TalonFX(turningMotorId, ModuleConstants.kCANivoreName);
        
        this.stopRequest = new DutyCycleOut(0);
        this.driveRequest = new DutyCycleOut(0);
        this.turnPositionRequest = new PositionDutyCycle(0);
        this.driveRequest.EnableFOC = true;
        this.turnPositionRequest.EnableFOC = true;
        this.turnPositionRequest.Slot = 0;
        
        this.driveVelocityRequest = new VelocityDutyCycle(0);
        this.driveVelocityRequest.EnableFOC = true;
        
        this.driveMotorID = driveMotorId;
        this.turnMotorID = turningMotorId;
        this.CANCoderID = CANCoderId;

        this.driveMotor.setInverted(invertDriveMotor);
        this.turnMotor.setInverted(invertTurningMotor);
        this.canCoder = new CANcoder(CANCoderId, ModuleConstants.kCANivoreName);
        
        this.desiredState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        
        TalonFXConfigurator driveConfigurator = driveMotor.getConfigurator();
        TalonFXConfigurator turnConfigurator = driveMotor.getConfigurator();
        
        CANcoderConfiguration ccConfigs = new CANcoderConfiguration();
        canCoder.getConfigurator().refresh(ccConfigs);
        ccConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        ccConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoder.getConfigurator().apply(ccConfigs);
        
        TalonFXConfiguration turnConfigs = new TalonFXConfiguration();
        turnConfigurator.refresh(turnConfigs);
        turnConfigs.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
        turnConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnConfigs.Feedback.SensorToMechanismRatio = 1.0;
        turnConfigs.Feedback.RotorToSensorRatio = 150.0 / 7.0;
        turnConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnConfigs.MotorOutput.DutyCycleNeutralDeadband =  ModuleConstants.kTurnMotorDeadband;
        turnConfigurator.apply(turnConfigs);
        
        this.drivePIDConfigs = new Slot0Configs();
        this.turnPIDConfigs = new Slot0Configs();
        refreshDrivePID();
        refreshTurnPID();

        MotorOutputConfigs driveConfigs = new MotorOutputConfigs();
        driveConfigurator.refresh(driveConfigs);
        driveConfigs.NeutralMode = NeutralModeValue.Coast;
        driveConfigs.DutyCycleNeutralDeadband =  ModuleConstants.kDriveMotorDeadband;
        driveConfigurator.apply(driveConfigs);
        
        FeedbackConfigs driveFeedbackConfigs = new FeedbackConfigs();
        driveConfigurator.refresh(driveFeedbackConfigs);
        driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        driveConfigurator.apply(driveFeedbackConfigs);
    }

    /**
     * Reset the CANCoder's relative encoder using its absolute encoder
     */
    public void resetEncoder() {
        canCoder.setPosition(canCoder.getAbsolutePosition().getValue());
        refreshDrivePID();

        ModuleConstants.ktunePID.loadPreferences();
        ModuleConstants.kPTurning.loadPreferences();
        ModuleConstants.kITurning.loadPreferences();
        ModuleConstants.kDTurning.loadPreferences();
    }

    public void refreshDrivePID() {
        driveMotor.getConfigurator().refresh(drivePIDConfigs);
        ModuleConstants.kPDrive.loadPreferences();
        ModuleConstants.kIDrive.loadPreferences();
        ModuleConstants.kDDrive.loadPreferences();
        ModuleConstants.kSDrive.loadPreferences();
        ModuleConstants.kVDrive.loadPreferences();
        drivePIDConfigs.kP = ModuleConstants.kPDrive.get();
        drivePIDConfigs.kI = ModuleConstants.kIDrive.get();
        drivePIDConfigs.kD = ModuleConstants.kDDrive.get();
        drivePIDConfigs.kS = ModuleConstants.kSDrive.get();
        drivePIDConfigs.kV = ModuleConstants.kVDrive.get();
        driveMotor.getConfigurator().apply(drivePIDConfigs);
    }

    public void refreshTurnPID() {
        turnMotor.getConfigurator().refresh(turnPIDConfigs);
        ModuleConstants.kPTurning.loadPreferences();
        ModuleConstants.kITurning.loadPreferences();
        ModuleConstants.kDTurning.loadPreferences();
        ModuleConstants.kFTurning.loadPreferences();
        turnPIDConfigs.kP = ModuleConstants.kPTurning.get();
        turnPIDConfigs.kI = ModuleConstants.kITurning.get();
        turnPIDConfigs.kD = ModuleConstants.kDTurning.get();
        turnPositionRequest.FeedForward = ModuleConstants.kFTurning.get();
        turnMotor.getConfigurator().apply(turnPIDConfigs);
    }

    /**
     * Set the percent output of both motors to zero.
     */
    public void stop() {
        this.stopRequest.Output = 0;
        driveMotor.setControl(stopRequest);
        turnMotor.setControl(stopRequest);

        this.desiredState = new SwerveModuleState(0, Rotation2d.fromRadians(getTurningPositionRadians()));
    }

    public void run() {
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getTurningPositionRadians()));

        desiredRotations = desiredState.angle.getRotations();

        double velocity = desiredState.speedMetersPerSecond / ModuleConstants.kMetersPerRevolution / ModuleConstants.kDriveMotorGearRatio;
        desiredVelocity = velocity;
        
        if (this.velocityControl) {
            // Velocity control to desired velocity
            driveMotor.setControl(driveVelocityRequest.withVelocity(velocity));
        } else {
            // Percent output estimation
            double output = desiredState.speedMetersPerSecond / SwerveDriveConstants.kPhysicalMaxSpeedMetersPerSecond;
            driveMotor.setControl(driveRequest.withOutput(output));
        }
        
        // Position Control to desired angle
        turnMotor.setControl(turnPositionRequest.withPosition(desiredRotations));   
    }

    public void resetDesiredAngle() {
        this.desiredRotations = 0;
    }
    
    //****************************** GETTERS ******************************/

    /**
     * Get the distance travelled by the motor in meters
     * @return Distance travelled by motor (in meters)
     */
    public double getDrivePosition() {
        return driveMotor.getRotorPosition().getValue()
            * ModuleConstants.kMetersPerRevolution
            * ModuleConstants.kDriveMotorGearRatio;
    }

    /**
     * Get the turning motor's CANCoder's angle
     * @return Angle in radians
     */
    public double getTurningPositionRadians() {
        double turningPosition = Math.toRadians(getTurningPositionDegrees());
        return turningPosition;
    }

    /**
     * Get the turning motor's CANCoder's angle
     * @return Angle in degrees
     */
    public double getTurningPositionDegrees() {
        double turningPosition = (canCoder.getPosition().getValue() % 1) * 360;
        return turningPosition;
    }

    /**
     * Get the velocity of the drive motor
     * @return Velocity of the drive motor (in meters / sec)
     */
    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValue() 
            * ModuleConstants.kMetersPerRevolution
            * ModuleConstants.kDriveMotorGearRatio;
    }

    /**
     * Get the velocity of the turning motor
     * @return Velocity of the turning motor (in rotations / sec)
     */
    public double getTurningVelocityRotations() {
        return canCoder.getVelocity().getValue();
    }

    /**
     * Return the current state (velocity and rotation) of the Swerve Module
     * @return This Swerve Module's State
     */
    public SwerveModuleState getState() {
        currState.speedMetersPerSecond = getDriveVelocity();
        currState.angle = Rotation2d.fromRadians(getTurningPositionRadians());
        return currState;
    }

    public SwerveModulePosition getPosition() {
        currPosition.distanceMeters = getDrivePosition();
        currPosition.angle = Rotation2d.fromRadians(getTurningPositionRadians());
        return currPosition;
    }

    //****************************** SETTERS ******************************/

    /**
     * Set the desired state of the Swerve Module and move towards it
     * @param state The desired state for this Swerve Module
     */
    public void setDesiredState(SwerveModuleState state, boolean withVelocityControl) {
        this.velocityControl = withVelocityControl;
        setDesiredState(state);
    }

    /**
     * Set the desired state of the Swerve Module and move towards it
     * @param state The desired state for this Swerve Module
     */
    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            state.speedMetersPerSecond = 0;
        }

        this.desiredState = state;
    }

    public void setVelocityControl(boolean velocityControlOn) {
        this.velocityControl = velocityControlOn;
    }

    public void toggleVelocityControl() {
        this.velocityControl = !velocityControl;
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL)  {
            return;
        }
        int moduleId = (driveMotorID / 10);
        ShuffleboardTab tab = Shuffleboard.getTab("Module " + moduleId);

        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
                tab.addNumber("Drive Motor Current", () -> driveMotor.getStatorCurrent().getValue());
                tab.addNumber("Turn Motor Current", () -> turnMotor.getStatorCurrent().getValue());
                tab.addNumber("Drive Motor Voltage", () -> (driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValue()));
                tab.addNumber("Turn Motor Voltage", () -> turnMotor.getSupplyVoltage().getValue());// ::getMotorOutputVoltage);
                tab.addNumber("Module velocity", this::getDriveVelocity);
                tab.addNumber("Desired Velocity", () -> this.desiredVelocity);
                tab.addNumber("Drive percent (motor controller)", () -> driveMotor.getDutyCycle().getValue());
                tab.addNumber("Drive percent (current)", () -> this.currentPercent);
                tab.addNumber("Turn angle percent", () -> turnMotor.getDutyCycle().getValue());
                tab.addBoolean("Velocity Control Enabled", () -> this.velocityControl);
            case MINIMAL:
                tab.addNumber("Module Direction", this::getTurningVelocityRotations);
                break;
        }

    }

    public void reportToSmartDashboard(LOG_LEVEL level) {
        currentAngle = Math.toDegrees(getTurningPositionRadians());
        switch (level) {
            case OFF:
                break;
            case ALL:
                SmartDashboard.putNumber("Drive Motor #" + driveMotorID + " Current", driveMotor.getStatorCurrent().getValue());
                SmartDashboard.putNumber("Turn Motor #" + turnMotorID + " Current", turnMotor.getStatorCurrent().getValue());
                SmartDashboard.putNumber("Drive Motor #" + driveMotorID + " Voltage", (driveMotor.getDutyCycle().getValue() * driveMotor.getSupplyVoltage().getValue()));
                SmartDashboard.putNumber("Turn Motor #" + turnMotorID + " Voltage", (turnMotor.getDutyCycle().getValue() * turnMotor.getSupplyVoltage().getValue()));
            case MEDIUM:
                SmartDashboard.putNumber("Module velocity #" + driveMotorID, getDriveVelocity());
                SmartDashboard.putNumber("Drive percent #" + driveMotorID, driveMotor.getDutyCycle().getValue());
                SmartDashboard.putNumber("Turn Angle #" + turnMotorID, currentAngle);
            case MINIMAL:
                break;
        }

    }

    /**
     * Enable or disable the break mode on the motors
     * @param breaking  Whether or not the motor should be on break mode
     */
    public void setBreak(boolean breaking) {
        NeutralModeValue mode = (breaking ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        
        MotorOutputConfigs turnConfigs = new MotorOutputConfigs();
        turnMotor.getConfigurator().refresh(turnConfigs);
        turnConfigs.NeutralMode = mode;
        turnMotor.getConfigurator().apply(turnConfigs);
        
        MotorOutputConfigs driveConfigs = new MotorOutputConfigs();
        driveMotor.getConfigurator().refresh(driveConfigs);
        driveConfigs.NeutralMode = mode;
        driveMotor.getConfigurator().apply(driveConfigs);
    }
}
