package frc.robot.subsystems.Arm.ArmIO;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.Arm.ArmConstants;

public interface ArmIO {

    public record ArmIOData(
        boolean isConnected,
        Angle position,
        AngularVelocity velocity,
        Voltage appliedVoltage,
        Current current,
        Temperature temprature
    ){}

    @AutoLog
    public class ArmIOInputs{
        
        public ArmIOData data = new ArmIOData(false, null, null, null, null, null);


    }

        public void updateInputs(ArmIOInputs inputs);

        public void runPercentPower(double percent);

        public void runVoltage(double voltage);

        public void runPosition(double position);

        public void stop();

        // public void setClosedLoopSlot(ClosedLoopSlot slot);

        public void setPID(double kP, double kI, double kD);

        public void setkS(double kS);

        public void setBrakeMode(IdleMode mode);

        public void reset();



    
    
}
