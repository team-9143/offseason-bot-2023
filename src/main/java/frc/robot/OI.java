package frc.robot;

import frc.robot.Constants.DeviceConstants;

import frc.robot.devices.CustomController;
import com.ctre.phoenix.sensors.Pigeon2;

public class OI {
  public final static CustomController driver_cntlr = new CustomController(DeviceConstants.kDriverCntlrPort);
  public final static CustomController operator_cntlr = new CustomController(DeviceConstants.kOperatorCntlrPort);

  // In proper orientation, Pigeon is flat and facing so that X-axis is forward
  /** Roll increases to the right, pitch to the front, and yaw counter-clockwise. */
  public final static Pigeon2 pigeon = new Pigeon2(DeviceConstants.kPigeonID) {
    @Override
    public double getPitch() {
      return -getPitch();
    };
  };
}