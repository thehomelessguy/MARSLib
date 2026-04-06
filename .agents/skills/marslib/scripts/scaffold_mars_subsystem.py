import argparse
import os

def scaffold(name, motor_type, is_rotary):
    base_dir = f"src/main/java/com/marslib/mechanisms/{name.lower()}"
    os.makedirs(base_dir, exist_ok=True)

    io_name = f"{name}IO"
    io_sim_name = f"{name}IOSim"
    io_hw_name = f"{name}IO{motor_type}"
    subsystem_name = f"{name}"

    pkg = f"com.marslib.mechanisms.{name.lower()}"

    io_content = f"""package {pkg};

import org.littletonrobotics.junction.AutoLog;

public interface {io_name} {{
    @AutoLog
    public static class {io_name}Inputs {{
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[] {{}};
        public double[] tempCelsius = new double[] {{}};
    }}

    public default void updateInputs({io_name}Inputs inputs) {{}}
    public default void setVoltage(double volts) {{}}
}}
"""

    io_hw_content = f"""package {pkg};

// Generated {motor_type} Hardware Interface
// TODO: Inject Phoenix 6 / REV APIs here

public class {io_hw_name} implements {io_name} {{
    public {io_hw_name}() {{
        // Initialize Hardware
    }}

    @Override
    public void updateInputs({io_name}Inputs inputs) {{
        // Read from Hardware
    }}

    @Override
    public void setVoltage(double volts) {{
        // Set voltage
    }}
}}
"""

    io_sim_content = f"""package {pkg};

import com.marslib.sim.MARSPhysicsWorld;
// Generated Physics Sim Interface

public class {io_sim_name} implements {io_name} {{
    public {io_sim_name}() {{
        // Initialize dyn4j constraints
    }}

    @Override
    public void updateInputs({io_name}Inputs inputs) {{
        // Read purely from simulation engine
    }}

    @Override
    public void setVoltage(double volts) {{
        // Set physics force
    }}
}}
"""

    sub_content = f"""package {pkg};

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class {subsystem_name} extends SubsystemBase {{
    private final {io_name} io;
    private final {io_name}InputsAutoLogged inputs = new {io_name}InputsAutoLogged();

    public {subsystem_name}({io_name} io) {{
        this.io = io;
    }}

    @Override
    public void periodic() {{
        io.updateInputs(inputs);
        Logger.processInputs("{subsystem_name}", inputs);
    }}
}}
"""

    def write_file(filename, content):
        path = os.path.join(base_dir, filename)
        with open(path, "w") as f:
            f.write(content)
        print(f"Created: {path}")

    write_file(f"{io_name}.java", io_content)
    write_file(f"{io_hw_name}.java", io_hw_content)
    write_file(f"{io_sim_name}.java", io_sim_content)
    write_file(f"{subsystem_name}.java", sub_content)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Scaffold MARSLib Subsystem")
    parser.add_argument("name", type=str, help="Name of the mechanism (e.g. Intake)")
    parser.add_argument("motor_type", choices=["TalonFX", "SparkMax"], help="Primary motor type")
    parser.add_argument("--rotary", action="store_true", help="Is this a rotary joint?")
    args = parser.parse_args()
    
    scaffold(args.name, args.motor_type, args.rotary)
