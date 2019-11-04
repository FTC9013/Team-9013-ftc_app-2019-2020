package org.firstinspires.ftc.teamcode;


import android.support.annotation.NonNull;
import android.support.annotation.Nullable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.meanIntegrate;
import static org.firstinspires.ftc.robotcore.external.navigation.NavUtil.plus;

public class AccelerationIntegrator9013 implements BNO055IMUCustom.AccelerationIntegrator
{
  //------------------------------------------------------------------------------------------
  // State
  //------------------------------------------------------------------------------------------

  BNO055IMU.Parameters parameters;
  private Position position;
  private Velocity velocity;
  private Acceleration previousAcceleration;


  public Position getPosition() { return this.position; }
  public Velocity getVelocity() { return this.velocity; }
  public Acceleration getAcceleration() { return this.previousAcceleration; }

  //------------------------------------------------------------------------------------------
  // Construction
  //------------------------------------------------------------------------------------------

  public AccelerationIntegrator9013()
  {
    this.parameters = null;
    this.position = new Position();
    this.velocity = new Velocity();
    this.previousAcceleration = null;
  }

  //------------------------------------------------------------------------------------------
  // Operations
  //------------------------------------------------------------------------------------------

  @Override public void initialize(@NonNull BNO055IMU.Parameters parameters,
                                   @Nullable Position initialPosition,
                                   @Nullable Velocity initialVelocity)
  {
    this.parameters = parameters;
    this.position = initialPosition != null ? initialPosition : this.position;
    this.velocity = initialVelocity != null ? initialVelocity : this.velocity;
    this.previousAcceleration = null;
  }

  // This runs less frequently than the acceleration filter.  It does the integration
  // into velocity and position.
  @Override public void update(Acceleration filteredAcceleration)
  {
    // We should always have valid filtered data.
    if (previousAcceleration != null && filteredAcceleration.acquisitionTime != 0)
    {
      Velocity velocityPrev = velocity;

      Velocity deltaVelocity = meanIntegrate(filteredAcceleration, previousAcceleration);
      velocity = plus(velocity, deltaVelocity);

      if (velocityPrev.acquisitionTime != 0) // make sure we have at least one velocity sample.
      {
        Position deltaPosition = meanIntegrate(velocity, velocityPrev);
        position = plus(position, deltaPosition);
      }

      if (parameters != null && parameters.loggingEnabled)
      {
        RobotLog.vv(parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s",
            (filteredAcceleration.acquisitionTime - previousAcceleration.acquisitionTime)*1e-9,
            filteredAcceleration, velocity, position);
      }
    }
    previousAcceleration = filteredAcceleration;
  }
}
