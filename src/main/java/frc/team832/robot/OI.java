package frc.team832.robot;

import frc.team832.lib.driverinput.controllers.Attack3;
import frc.team832.lib.driverinput.controllers.Extreme3DPro;
import frc.team832.lib.driverinput.oi.*;

@SuppressWarnings("WeakerAccess")
public class OI {

    public final DriverOI driverOI;

    public final Attack3 leftStick;
    public final Extreme3DPro rightStick;

    public OI() {
//        driverOI = new XboxDriverOI();
         driverOI = new SticksDriverOI();

         leftStick = (Attack3)driverOI.getFirstController().get();
         rightStick = (Extreme3DPro)driverOI.getSecondController().get();

        // do commands here
    }
}