#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
//
//  cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
//  cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
//  cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
//  cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right
//
//
  

//
//    fn = k_f * self.omega[n]**2
//
//    def tau_1(self):
//    tau = self.k_m * self.omega[0]**2
//    return tau
//
//    def tau_2(self):
//    tau = -self.k_m * self.omega[1]**2
//    return tau
//
//    def tau_3(self):
//    tau = self.k_m * self.omega[2]**2
//    return tau
//
//    def tau_4(self):
//    tau = -self.k_m * self.omega[3]**2
//    return tau
    

//    τx=(F1+F4−F2−F3)l
//    τy=(F1+F2−F3−F4)l
//    τz=τ1+τ2+τ3+τ4
  
//    k_m/k_f = - kappa
    
//    Fn = kappa * omega[n]^2
//    collThrustCmd = F1+F2+F3+F4 = a
//    τx/l =          F1−F2−F3+F4 = b
//    τy/l =          F1+F2−F3−F4 = c
//    τz/-kappa =     F1-F2+F3-F4 = d
  
  float l = L/sqrt(2);
  float a = collThrustCmd;
  float b = momentCmd.x/l;
  float c = momentCmd.y/l;
  float d = -momentCmd.z/kappa;



  float F1 = (a + b + c +d)/4;
  float F2 = (a + c )/2 - F1;
  float F3 = (a + b )/2 - F1;
  float F4 = a - F1 - F2 - F3;
  cmd.desiredThrustsN[0] = CONSTRAIN(F1, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[1] = CONSTRAIN(F2, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[2] = CONSTRAIN(F3, minMotorThrust, maxMotorThrust);
  cmd.desiredThrustsN[3] = CONSTRAIN(F4, minMotorThrust, maxMotorThrust);
  

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
//  $p_{\text{error}} = p_c - p$
//
//  $\bar{u}_p= k_{p-p} p_{\text{error}}$
//
//  $q_{\text{error}} = q_c - q$
//
//  $\bar{u}_q= k_{p-q} q_{\text{error}}$
//
//  $r_{\text{error}} = r_c - r$
//
//  $\bar{u}_r= k_{p-r} r_{\text{error}}$

//  float p_actual = pqr.x;
//  float q_actual = pqr.y;
//  float r_actual = pqr.z;
//  float p_c = pqrCmd.x;
//  float q_c = pqrCmd.y;
//  float r_c = pqrCmd.z;
//  float k_p_p = kpPQR.x;
//  float k_p_q = kpPQR.y;
//  float k_p_r = kpPQR.z;
//
//  float p_err=p_c - p_actual;
//  float u_bar_p = k_p_p * p_err;
//
//  float q_err= q_c - q_actual;
//  float u_bar_q = k_p_q * q_err;
//
//  float r_err= r_c - r_actual;
//  float u_bar_r = k_p_r * r_err;
//
//
//  momentCmd = V3F(Ixx * u_bar_p, Iyy * u_bar_q, Izz * u_bar_r);

  V3F errorPQR = (pqrCmd - pqr);
  momentCmd = kpPQR * errorPQR;
  momentCmd.x *= Ixx;
  momentCmd.y *= Iyy;
  momentCmd.z *= Izz;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
//  python code
//  b_x = rot_mat[0,2]
//  b_x_err = b_x_c_target - b_x
//  b_x_p_term = self.k_p_roll * b_x_err
//
//  b_y = rot_mat[1,2]
//  b_y_err = b_y_c_target - b_y
//  b_y_p_term = self.k_p_pitch * b_y_err
//
//  b_x_commanded_dot = b_x_p_term
//  b_y_commanded_dot = b_y_p_term
//
//  rot_mat1=np.array([[rot_mat[1,0],-rot_mat[0,0]],[rot_mat[1,1],-rot_mat[0,1]]])/rot_mat[2,2]
//
//  rot_rate = np.matmul(rot_mat1,np.array([b_x_commanded_dot,b_y_commanded_dot]).T)
//  p_c = rot_rate[0]
//  q_c = rot_rate[1]

  float a_c = -collThrustCmd / mass;
  
  float b_x = R(0, 2);
  float b_x_c_target = accelCmd.x / a_c;
  b_x_c_target = CONSTRAIN(b_x_c_target, -maxTiltAngle, maxTiltAngle);
  float b_x_err = b_x_c_target - b_x;
  float b_x_p_term = kpBank * b_x_err;
  
  float b_y = R(1, 2);
  float b_y_c_target = accelCmd.y / a_c;
  b_y_c_target = CONSTRAIN(b_y_c_target, -maxTiltAngle, maxTiltAngle);
  float b_y_err = b_y_c_target - b_y;
  float b_y_p_term = kpBank * b_y_err;

  pqrCmd.x = ( R(1,0) * b_x_p_term - R(0,0) * b_y_p_term/ R(2,2));
  pqrCmd.y = ( R(1,1) * b_x_p_term - R(0,1) * b_y_p_term / R(2,2));
  
//  float accel = -collThrustCmd / mass;
//
//  float b_x = R(0, 2);
//  float b_x_c = accelCmd.x / accel;
//  float b_x_c_constrained = CONSTRAIN(b_x_c, -maxTiltAngle, maxTiltAngle);
//  float b_x_err = b_x_c_constrained - b_x;
//  float b_x_p_term = kpBank * b_x_err;
//
//  float b_y = R(1, 2);
//  float b_y_c = accelCmd.y / accel;
//  float b_y_c_constrained = CONSTRAIN(b_y_c, -maxTiltAngle, maxTiltAngle);
//  float b_y_err = b_y_c_constrained - b_y;
//  float b_y_p_term = kpBank * b_y_err;
//
//  float p_cmd = (1 / R(2, 2)) * (R(1, 0) * b_x_p_term + -R(0, 0) * b_y_p_term);
//  float q_cmd = (1 / R(2, 2)) * (R(1, 1) * b_x_p_term + -R(0, 1) * b_y_p_term);
//
//  pqrCmd.x = p_cmd;
//  pqrCmd.y = q_cmd;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
//  python code
//  z_err = z_target - z_actual
//  z_err_dot = z_dot_target - z_dot_actual
//  b_z = rot_mat[2,2]
//
//  p_term = self.z_k_p * z_err
//  d_term = self.z_k_d * z_err_dot
//
//  u_1_bar = p_term + d_term + z_dot_dot_target
//
//  c = (u_1_bar - self.g)/b_z
  float z_err = posZCmd - posZ;
  float z_err_dot = velZCmd - velZ;
  float b_z = R(2,2);
  
  float p_term = kpPosZ * z_err;
  float d_term = kpVelZ * z_err_dot;
  
  integratedAltitudeError += z_err *dt;
  float i_term = integratedAltitudeError * KiPosZ;


  float u_1_bar = p_term + d_term + i_term +  accelZCmd;
//  float u_1_bar = p_term + d_term + accelZCmd;
  
  float c  = (u_1_bar - 9.81f)/b_z;
  thrust = - mass * CONSTRAIN(c, -maxAscentRate/dt, maxAscentRate/dt);
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

//  python code
//  x_err = x_target - x_actual
//  x_err_dot = x_dot_target - x_dot_actual
//
//  p_term_x = self.x_k_p * x_err
//  d_term_x = self.x_k_d * x_err_dot
//
//  x_dot_dot_command = p_term_x + d_term_x + x_dot_dot_target
//
//  b_x_c = x_dot_dot_command/c
//
//
//  y_err = y_target - y_actual
//  y_err_dot = y_dot_target - y_dot_actual
//
//  p_term_y = self.y_k_p * y_err
//  d_term_y = self.y_k_d * y_err_dot
//
//  y_dot_dot_command = p_term_y + d_term_y + y_dot_dot_target
//
//  b_y_c = y_dot_dot_command/c
  
  if ( velCmd.mag() > maxSpeedXY ) {
    velCmd = velCmd.norm() * maxSpeedXY;
  }

  V3F err = posCmd - pos;
  V3F err_dot = velCmd - vel;
  float p_term_x =  kpPosXY * err.x;
  float d_term_x = kpVelXY * err_dot.x;
  float x_dot_dot_command = p_term_x + d_term_x + accelCmdFF.x;

  float p_term_y =  kpPosXY * err.y;
  float d_term_y = kpVelXY * err_dot.y;
  float y_dot_dot_command = p_term_y + d_term_y + accelCmdFF.y;

  accelCmd.x += x_dot_dot_command;
  accelCmd.y += y_dot_dot_command;

  if( accelCmd.mag() > maxSpeedXY) {
    accelCmd = accelCmd.norm() * maxAccelXY;
  }
  
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

//  python code
//  psi_err = psi_target - psi_actual
//  r_c = self.k_p_yaw * psi_err

  float psi_err = yawCmd - yaw;
  yawRateCmd =  kpYaw * psi_err;
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
