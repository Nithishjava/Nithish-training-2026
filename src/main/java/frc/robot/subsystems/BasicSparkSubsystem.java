package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class BasicSparkSubsystem {
    //Constrants that are normally stored in a seperate Constants class.

    private final int sparkID = 0;
    private final boolean inverted = true;
    private final int currentLimit = 60;

    private final double slot0kP = 0.1;
    private final double slot0kI = 0.0;
    private final double slot0kD = 0.0;

    private final double slot0Ks = 0.1;

    private final double slot1kP = 0.1;
    private final double slot1kI = 0.0;
    private final double slot1kD = 0.0;

    private final double slot1Ks = 0.1;
    private final double slot1kV = 0.1;

    // Actual Motor programming objects
    private SparkMax motor;
    private SparkMaxConfig config;

    private SparkClosedLoopController controller;
    private ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;

    private boolean isBrakeMode = true;

    public BasicSparkSubsystem() {
        motor = new SparkMax(sparkID, MotorType.kBrushless);

        controller = motor.getClosedLoopController();

        // General Config
        config
            .inverted(inverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(currentLimit)
            .voltageCompensation(12);
        
        config.encoder
            .positionConversionFactor(360)
            .velocityConversionFactor(360/60);
            
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(
                slot0kP, 
                currentLimit, 
                slot0kD, 
                ClosedLoopSlot.kSlot0
                )
            .pid(
                slot1kP,
                slot1kI,
                slot1kD,
                ClosedLoopSlot.kSlot1
            );

            config.closedLoop.feedForward
                .kS(slot0Ks, ClosedLoopSlot.kSlot0)
                .kS(slot1Ks, ClosedLoopSlot.kSlot1)
                .kV(slot1kV, ClosedLoopSlot.kSlot1);;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
            
            config.signals
                .appliedOutputPeriodMs(20) // See what voltage motor is using
                .outputCurrentPeriodMs(100) // Catch current spike
                .busVoltagePeriodMs(100) // Monitor battery voltage
                .faultsAlwaysOn(true)
                .faultsPeriodMs(1000) // Updates once a second, faults don't need hihg update rate
                .iAccumulationAlwaysOn(true) // tracks the integral accumulator for PID
                .motorTemperaturePeriodMs(1000)
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs(20)
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20) // Updates at robot loop rate
                .warningsAlwaysOn(true)
                .warningsPeriodMs(1000); // Updates once a second, warnings don't need high update rates

            
            motor.clearFaults();
            // Apply config to the motor (persist safe parameters by default)
            motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void runPercentOutput(double percentOutput){
        motor.set(percentOutput);
    }

    public void runVoltage(double voltageVolts){
        motor.setVoltage(voltageVolts);
    }

    public void runPosition(double position){
        // slot 0 for voltage
        setPIDSlot(ClosedLoopSlot.kSlot0);
        controller.setSetpoint(position, ControlType.kPosition, closedLoopSlot);
    }

    public void runVelocity(double velocity){
        // slot 1 for velocity
        setPIDSlot(ClosedLoopSlot.kSlot1);
        controller.setSetpoint(velocity, ControlType.kPosition, closedLoopSlot);
    }

    public void stop(){
        motor.stopMotor();
    }

    public void setPIDSlot(ClosedLoopSlot slot){
        closedLoopSlot = slot;
    }

    public void setBrakeMode(boolean brake){
        // Only change the brake mode if it is different from the current mode
        // Setting a new configuration takes cycle time on the robot, so we want to avoid it doing unessarily.
        if(isBrakeMode != brake){
            SparkMaxConfig configuration = new SparkMaxConfig();
            configuration.idleMode(brake? SparkMaxConfig.IdleMode.kBrake:SparkMaxConfig.IdleMode.kBrake);

            motor.configure(configuration, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            isBrakeMode = brake;
        }
    }

    public void reset(){
        motor.clearFaults();
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

}
