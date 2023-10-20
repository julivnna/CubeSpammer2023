package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.NerdyMath;
import frc.robot.util.filters.ExponentialSmoothingFilter;

public class Wrist extends SubsystemBase implements Reportable {
    private TalonFX wrist;
    private int targetTicks = WristConstants.kWristStow;
    private int targetIntakeTicks = WristConstants.kWristLowPickup;
    private Supplier<Double> manualInput;

    private boolean motionMagicOn = true;

    public BooleanSupplier atTargetPosition = () -> false;
    // private TalonSRX leftEncoder;
    private ExponentialSmoothingFilter joystickFilter = new ExponentialSmoothingFilter(WristConstants.kLowPassAlpha);

    public Wrist(Supplier<Double> manualInput) {
        wrist = new TalonFX(WristConstants.kWristID);
        // leftEncoder = new TalonSRX(WristConstants.kLeftEncoderID);
        init();
        resetEncoders();
        this.manualInput = manualInput;
        wrist.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 30, 0.1));
    }

    @Override
    public void periodic() {
        if (motionMagicOn) {

            // if (!DriverStation.isTest()) {
            //     moveWristMotionMagic();
            // } else {
                wrist.set(ControlMode.PercentOutput, 0);
            // }

        } else {

            if (Math.abs(manualInput.get()) > ControllerConstants.kDeadband) {
                wrist.set(ControlMode.PercentOutput, 0.5 * manualInput.get());
            } else {
                wrist.set(ControlMode.PercentOutput, 0);
            }

        }
    }

    public void init() {
        wrist.setNeutralMode(NeutralMode.Brake);
        wrist.setInverted(true);
        // leftEncoder.setInverted(false);
        // leftEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 1000);
        // leftEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 1, 1000);
        //test
        zeroEncodersStow();
        wrist.config_kP(0, WristConstants.kWristP.get());
        wrist.config_kI(0, WristConstants.kWristI.get());
        wrist.config_kD(0, WristConstants.kWristD.get());
        wrist.config_kF(0, WristConstants.kWristF.get());

        wrist.configMotionCruiseVelocity(WristConstants.kWristCruiseVelocity);
        wrist.configMotionAcceleration(WristConstants.kWristMotionAcceleration);

    }

    // Use during a match to re-enable motion magic and turn off joystick input
    public void resetToStowPosition() {
        toggleMotionMagic(true);
        zeroEncodersStow();
        this.targetIntakeTicks = WristConstants.kWristStow;
    }

    public void toggleMotionMagic(boolean motionMagicOn) {
        this.motionMagicOn = motionMagicOn;
    }

    public void holdPosition() {
        this.targetTicks = (int) wrist.getSelectedSensorPosition();
    }

    public void resetEncoders(){
        // double absoluteTicks = leftEncoder.getSelectedSensorPosition(0);
        // wrist.setSelectedSensorPosition(absoluteTicks * WristConstants.kFalconTicksPerAbsoluteTicks, 0, 100);
        wrist.setSelectedSensorPosition(WristConstants.kWristStow);
        WristConstants.kWristP.loadPreferences();
        WristConstants.kWristI.loadPreferences();
        WristConstants.kWristD.loadPreferences();
        WristConstants.kWristF.loadPreferences();
        WristConstants.kWristFF.loadPreferences();
        wrist.config_kP(0, WristConstants.kWristP.get());
        wrist.config_kI(0, WristConstants.kWristI.get());
        wrist.config_kD(0, WristConstants.kWristD.get());
        wrist.config_kF(0, WristConstants.kWristF.get());
    }

    public void moveWristMotionMagicJoystick(double joystickInput) {
        if (joystickInput < -0.1 || joystickInput > 0.1) {
            int tickChange = (int) (WristConstants.kJoystickScale * joystickInput);
            int currentTicks = (int) wrist.getSelectedSensorPosition();

            // tickChange = (int) joystickFilter.calculate(tickChange);

            targetTicks = currentTicks + tickChange;
            targetTicks = (int) NerdyMath.clamp(targetTicks, WristConstants.kWristLowerLimit, WristConstants.kWristUpperLimit);
        }
        else {
            joystickFilter.calculate(0);
        }

        setTargetTicks(targetTicks);
    }


    public double getWristAngle() {
        double ticks = wrist.getSelectedSensorPosition(0);
        double angle = ticks * WristConstants.kDegreesPerTick % 360;
        return angle;
    }

    public CommandBase changeIntakeTargetTicks(int increment) {
        return Commands.runOnce(() -> {targetIntakeTicks += increment;});
    }

    public CommandBase resetIntakeTargetTicks() {
        return Commands.runOnce(() -> {targetIntakeTicks = WristConstants.kWristLowPickup;});
    }

    public void setIntakeTargetTicks() {
        setTargetTicks(targetIntakeTicks);
    }

    public int getIntakeTargetTicks() {
        return targetIntakeTicks;
    }
    
    public void setNormalTargetTicks() {
        setTargetTicks(targetTicks);
    }


    // public void changeTargetTicks(int increment) {
    //     targetPickupTicks += increment;
    // }

    public double getWristAngleRadians() {
        return Math.toRadians(getWristAngle());
    }

    public void moveWristMotionMagic() {
        double ff = WristConstants.kWristFF.get() * Math.cos(getWristAngleRadians());
        wrist.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);
    }

    public CommandBase motionMagicCommand(int position) {
        return Commands.runOnce(() -> {
            wrist.configMotionCruiseVelocity(WristConstants.kWristCruiseVelocity);
            wrist.configMotionAcceleration(WristConstants.kWristMotionAcceleration);
            setTargetTicks(position);
        });
    }

    // public CommandBase motionMagicCommand(Supplier<Integer> position) {
    //     return Commands.run(() -> 
    //         () -> setTargetTicks(position.get())
    //     );
    // }

    public CommandBase intake() {
        return Commands.run(
            () -> setTargetTicks(targetIntakeTicks)
        );
    }

    public void setTargetTicks(int targetTicks) {
        toggleMotionMagic(true);
        this.targetTicks = targetTicks;
    }

    @Override
    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                SmartDashboard.putNumber("Wrist Motor Output", wrist.getMotorOutputPercent());
                SmartDashboard.putNumber("Wrist Angle", Math.toDegrees(getWristAngle()));
                SmartDashboard.putNumber("Wrist Velocity", wrist.getSelectedSensorVelocity());
            case MEDIUM:
                SmartDashboard.putNumber("Wrist Current", wrist.getStatorCurrent());
                SmartDashboard.putNumber("Wrist Voltage", wrist.getMotorOutputVoltage());
            case MINIMAL:
                SmartDashboard.putNumber("Wrist Ticks", wrist.getSelectedSensorPosition());
                SmartDashboard.putNumber("Target Wrist Ticks", targetTicks);
                break;
        }
        
    }

    public void zeroEncodersStow() {
        // leftEncoder.setSelectedSensorPosition(WristConstants.kWristStowPowerOff / WristConstants.kFalconTicksPerAbsoluteTicks, 1, 1000);
        // leftEncoder.setSelectedSensorPosition(WristConstants.kWristStowPowerOff / WristConstants.kFalconTicksPerAbsoluteTicks, 0, 1000);
        wrist.setSelectedSensorPosition(WristConstants.kWristStowPowerOff, 0, 1000);
    }

    @Override
    public void initShuffleboard(LOG_LEVEL level) { 
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Wrist");
        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addNumber("Motor Output", wrist::getMotorOutputPercent);
                tab.addString("Control Mode", wrist.getControlMode()::toString);
                tab.add("Zero wrist angle", Commands.runOnce(() -> {
                    // leftEncoder.setSelectedSensorPosition(0, 1, 1000);
                    // leftEncoder.setSelectedSensorPosition(0, 0, 1000);
                    wrist.setSelectedSensorPosition(0, 0, 1000);
                    resetEncoders();
                }));
                // tab.addNumber("Wrist Target Velocity", wrist::getActiveTrajectoryVelocity); 
                // tab.addNumber("Closed loop error", wrist::getClosedLoopError);
                tab.add("Stow wrist angle", Commands.runOnce(() -> {
                    // leftEncoder.setSelectedSensorPosition(WristConstants.kWristStowPowerOff / WristConstants.kFalconTicksPerAbsoluteTicks, 1, 1000);
                    // leftEncoder.setSelectedSensorPosition(WristConstants.kWristStowPowerOff / WristConstants.kFalconTicksPerAbsoluteTicks, 0, 1000);
                    wrist.setSelectedSensorPosition(WristConstants.kWristStowPowerOff, 0, 1000);
                }));

            case MEDIUM:
                tab.addNumber("Wrist Stator Current", wrist::getStatorCurrent);
                tab.addNumber("Wrist Supply Curremt", wrist::getSupplyCurrent);
                tab.addNumber("Wrist Velocity", wrist::getSelectedSensorVelocity);
                tab.addNumber("Wrist Voltage", wrist::getMotorOutputVoltage);
                tab.addNumber("Wrist Percent Output", wrist::getMotorOutputPercent);

            case MINIMAL:
                tab.addNumber("Current Wrist Ticks", wrist::getSelectedSensorPosition);
                // tab.addNumber("Current Wrist Absolute Ticks", () -> leftEncoder.getSelectedSensorPosition(0));
                // tab.addNumber("Current Wrist Absolute Angle", () -> leftEncoder.getSelectedSensorPosition(0) / 4096 * 360);
                tab.addNumber("Target Wrist Ticks", () -> targetTicks);
                tab.addNumber("MotionMagic Velocity", wrist::getActiveTrajectoryVelocity);
                tab.addNumber("MotionMagic Position", wrist::getActiveTrajectoryPosition);
                tab.addBoolean("At target position", atTargetPosition);
                tab.addNumber("Current Wrist Angle", this::getWristAngle);
                break;
        }
        }
        
    }

