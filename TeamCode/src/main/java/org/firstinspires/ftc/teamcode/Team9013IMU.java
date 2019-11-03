package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMUImpl;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.UserConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import static com.qualcomm.robotcore.hardware.ControlSystem.REV_HUB;


@I2cDeviceType
//@DeviceProperties(name = "@string/lynx_embedded_imu_name", xmlTag = LynxConstants.EMBEDDED_IMU_XML_TAG, description = "@string/lynx_embedded_imu_description", builtIn = true, compatibleControlSystems = REV_HUB)
@DeviceProperties(name = "Team9013IMU", xmlTag = "Team9013IMU", description = "Team9013IMU", compatibleControlSystems = REV_HUB)
public class Team9013IMU extends BNO055IMUCustom
{
  //----------------------------------------------------------------------------------------------
  // Construction
  //----------------------------------------------------------------------------------------------

  /**
   * This constructor is used by {@link UserConfigurationType#createInstance(I2cController, int)}
   * @see UserConfigurationType#createInstance(I2cController, int)
   * @see I2cDeviceType
   */
  public Team9013IMU(I2cDeviceSynch deviceClient)
  {
    super(deviceClient);
  }

  @Override public String getDeviceName()
  {
    return AppUtil.getDefContext().getString(com.qualcomm.robotcore.R.string.lynx_embedded_imu_name);
  }

  @Override public Manufacturer getManufacturer()
  {
    return Manufacturer.Lynx;
  }
}
