package frc.team832.robot;

import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.OperatorInterface;

@SuppressWarnings("WeakerAccess")
public class OI {

    public final DriverOI driverOI;

    public OI() {
        driverOI = OperatorInterface.getDriverOIForAttached();
        // do commands here
    }
}