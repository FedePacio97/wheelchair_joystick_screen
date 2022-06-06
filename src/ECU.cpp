#include<Arduino.h>
#include"Joystick.h"
#include"ECU.h"

#define DEBUG_LEVEL 0

//int ECU::MAX_CURRENT = 2000;

float ECU::MAX_CURRENT = 15.0;

            //Initialization list
ECU::ECU() : joystick(JOYSTICK_Y_PIN,JOYSTICK_X_PIN) /*, Accelerometer(), [...] */{
  angular_speed_rear_wheels.omega_left = angular_speed_rear_wheels.omega_right = 0;
  wheelchair_reference_speed.angular = wheelchair_reference_speed.linear = 0;
  reference_CURRENT_rear_wheels.CURRENT_lx = reference_CURRENT_rear_wheels.CURRENT_rx = 0;
  //MAX_CURRENT = DEFAULT_MAX_CURRENT;
}

/*
void ECU::update_reference_CURRENT_rear_wheels(){
  joystick.update_charthesian_coordinates();
  int degree = joystick.get_discretized_degree();
  float module = joystick.get_module();
  
  if(degree >= 0 && degree <=90 ){ //QUADRANT 1 [0,90] 
    reference_CURRENT_rear_wheels.CURRENT_rx = module * map(degree, 0, 90, -1 * MAX_CURRENT, MAX_CURRENT );
    reference_CURRENT_rear_wheels.CURRENT_lx = module * MAX_CURRENT;
    return;
  }
  if(degree >= 90 && degree <= 180){ //QUADRANT 2 ]90,180]
      reference_CURRENT_rear_wheels.CURRENT_rx = module * MAX_CURRENT;
      reference_CURRENT_rear_wheels.CURRENT_lx = module * map(degree,180, 90,-1 * MAX_CURRENT, MAX_CURRENT);
      return;
  }
  if(degree >= -180 && degree <= -90){ //QUADRANT 3 [-180,-90]
      reference_CURRENT_rear_wheels.CURRENT_rx = module * map(degree,-90, -180,-1 * MAX_CURRENT, MAX_CURRENT);
      reference_CURRENT_rear_wheels.CURRENT_lx = -1 * module * MAX_CURRENT;   
      return;
  }
  if(degree > -90 && degree < 0){ //QUADRANT 4 [-90,0[
      reference_CURRENT_rear_wheels.CURRENT_rx = -1 * module * MAX_CURRENT;
      reference_CURRENT_rear_wheels.CURRENT_lx = module * map(degree,0, -90,MAX_CURRENT, -1 * MAX_CURRENT);
  }
  

}

int ECU::get_reference_CURRENT_lx(){
  return reference_CURRENT_rear_wheels.CURRENT_lx;
}

int ECU::get_reference_CURRENT_rx(){
  return reference_CURRENT_rear_wheels.CURRENT_rx;
}*/

void ECU::update_angular_reference_speed_rear_wheels(){
  joystick.update_charthesian_coordinates();
  int degree = joystick.get_discretized_degree();
  float module = joystick.get_module();
  
  #if DEBUG_LEVEL > 1
    Serial.printf("Joystick module -> %f\t degree -> %d\n",module,degree);
  #endif

  if(degree >= 0 && degree <=90 ){ //QUADRANT 1 [0,90] 
    angular_speed_rear_wheels.omega_right = module * map(degree, 0, 90, -1 * MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED );
    angular_speed_rear_wheels.omega_left = module * MAX_ANGULAR_SPEED;
    return;
  }
  if(degree >= 90 && degree <= 180){ //QUADRANT 2 ]90,180]
      angular_speed_rear_wheels.omega_right = module * MAX_ANGULAR_SPEED;
      angular_speed_rear_wheels.omega_left = module * map(degree,180, 90,-1 * MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
      return;
  }
  if(degree >= -180 && degree <= -90){ //QUADRANT 3 [-180,-90]
      angular_speed_rear_wheels.omega_right = module * map(degree,-90, -180,-1 * MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
      angular_speed_rear_wheels.omega_left = -1 * module * MAX_ANGULAR_SPEED;   
      return;
  }
  if(degree > -90 && degree < 0){ //QUADRANT 4 [-90,0[
      angular_speed_rear_wheels.omega_right = -1 * module * MAX_ANGULAR_SPEED;
      angular_speed_rear_wheels.omega_left = module * map(degree,0, -90,MAX_ANGULAR_SPEED, -1 * MAX_ANGULAR_SPEED);
  }
  

}

//Usage
    //ECU ecu = ECU(..);
    //ecu.update_wheelchair_reference_speed()
    //float reference_linear_speed = ecu.get_reference_linear_speed();
    //float reference_angular_speed = ecu.get_reference_angular_speed();

float ECU::get_reference_angular_speed(){
  return wheelchair_reference_speed.angular;
}

float ECU::get_reference_linear_speed(){
  return wheelchair_reference_speed.linear;
}

float ECU::get_reference_CURRENT_lx(){
  return reference_CURRENT_rear_wheels.CURRENT_lx;
}

float ECU::get_reference_CURRENT_rx(){
  return reference_CURRENT_rear_wheels.CURRENT_rx;
}

void ECU::update_wheelchair_reference_speed(){
  update_angular_reference_speed_rear_wheels(); //Update angular_speed_rear_wheels struct

  float v_ref_without_limit = (angular_speed_rear_wheels.omega_right + angular_speed_rear_wheels.omega_left) * R / 2; 
  float omega_ref_without_limit = (angular_speed_rear_wheels.omega_right - angular_speed_rear_wheels.omega_left) * R / d;

  #if DEBUG_LEVEL > 1
    Serial.printf("omega_ref_without_limit -> %f\n",omega_ref_without_limit);
    Serial.printf("v_ref_without_limit -> %f\n",v_ref_without_limit);
  #endif

  //limit_wheelchair_reference_speed();
  float v_ref_with_limit = mapFloat(v_ref_without_limit, -1 * V_MAX_WITHOUT_LIMIT, V_MAX_WITHOUT_LIMIT,-1 * V_MAX_LIMIT, V_MAX_LIMIT ); // = v_ref_without_limit * V_MAX_LIMIT / V_MAX_WITHOUT_LIMIT
  float omega_ref_with_limit = mapFloat(omega_ref_without_limit,-1 * OMEGA_MAX_WITHOUT_LIMIT, OMEGA_MAX_WITHOUT_LIMIT,-1 * OMEGA_MAX_LIMIT, OMEGA_MAX_LIMIT);

  wheelchair_reference_speed.linear = v_ref_with_limit;
  wheelchair_reference_speed.angular = omega_ref_with_limit;

}

void ECU::update_CURRENT_reference_rear_wheels(Stability_message &stability_info){
  joystick.update_charthesian_coordinates();
  int degree = joystick.get_discretized_degree();
  float module = joystick.get_module();
  
  #if DEBUG_LEVEL > 1
    Serial.printf("Joystick module -> %f\t degree -> %d\n",module,degree);
  #endif

  // float K = 0.5;

  // if(degree >= 0 && degree <=90 ){ //QUADRANT 1 [0,90] 
  //   //angular_speed_rear_wheels.omega_right = module * map(degree, 0, 90, -1 * MAX_CURRENT, MAX_CURRENT );
  //  // angular_speed_rear_wheels.omega_right = module * map(degree, 0, 90, 0, MAX_CURRENT );
  //  angular_speed_rear_wheels.omega_right = module * map(degree, 0, 90, -0.5 * MAX_CURRENT, MAX_CURRENT );
  //   angular_speed_rear_wheels.omega_left = module * MAX_CURRENT;

  //   // angular_speed_rear_wheels.omega_right = module * (MAX_CURRENT * (1- (90 - degree) / 90 ) + K * MAX_CURRENT * ( (90 - degree) / 90 ));
  //   // angular_speed_rear_wheels.omega_left = module * (MAX_CURRENT * (1- (90 - degree) / 90 ) - K * MAX_CURRENT * ( (90 - degree) / 90 ));
  //   //return;
  // }
  // if(degree >= 90 && degree <= 180){ //QUADRANT 2 ]90,180]
  //     angular_speed_rear_wheels.omega_right = module * MAX_CURRENT;
  //     //angular_speed_rear_wheels.omega_left = module * map(degree,180, 90,-1 * MAX_CURRENT, MAX_CURRENT);
  //    // angular_speed_rear_wheels.omega_left = module * map(degree,180, 90,0, MAX_CURRENT);
  //    angular_speed_rear_wheels.omega_left = module * map(degree,180, 90,-0.5 * MAX_CURRENT, MAX_CURRENT);

  //     // angular_speed_rear_wheels.omega_right = module * (MAX_CURRENT * ( (180 - degree) / 90 ) + K * MAX_CURRENT * (1 - (180 - degree) / 90 ));
  //     // angular_speed_rear_wheels.omega_left = module * (MAX_CURRENT * ( (180 - degree) / 90 ) - K * MAX_CURRENT * (1 - (180 - degree) / 90 ));
  //     //return;
  // }
  // if(degree >= -180 && degree <= -90){ //QUADRANT 3 [-180,-90]
  //   //  angular_speed_rear_wheels.omega_right = module * map(degree,-90, -180,-1 * MAX_CURRENT, MAX_CURRENT);
  //   angular_speed_rear_wheels.omega_right = module * map(degree,-90, -180,-1 * MAX_CURRENT, 0.5*MAX_CURRENT);
  //     angular_speed_rear_wheels.omega_left = -1 * module * MAX_CURRENT;   
  //     // angular_speed_rear_wheels.omega_right = module * (-1 * MAX_CURRENT * ( (270 - (360 + degree)) / 90 ) + K * MAX_CURRENT * (1 - (270 - (360 + degree)) / 90 ));
  //     // angular_speed_rear_wheels.omega_left = module * (-1 * MAX_CURRENT * ( (270 - (360 + degree) ) / 90 ) - K * MAX_CURRENT * (1 - (270 - (360 + degree)) / 90 ));
  //     //return;
  // }
  // if(degree > -90 && degree < 0){ //QUADRANT 4 [-90,0[
  //     angular_speed_rear_wheels.omega_right = -1 * module * MAX_CURRENT;
  //     //angular_speed_rear_wheels.omega_left = module * map(degree,0, -90,MAX_CURRENT, -1 * MAX_CURRENT);
  //     angular_speed_rear_wheels.omega_left = module * map(degree,0, -90,0.5*MAX_CURRENT, -1 * MAX_CURRENT);
  //     // angular_speed_rear_wheels.omega_right = module * (-1 * MAX_CURRENT * ( (360 - (360 + degree)) / 90 ) - K * MAX_CURRENT * (1 - (360 - (360 + degree)) / 90 ));
  //     // angular_speed_rear_wheels.omega_left = module * (-1 * MAX_CURRENT * ( (360 - (360 + degree) ) / 90 ) + K * MAX_CURRENT * (1 - (360 - (360 + degree)) / 90 ));
  // }

  float degtorad=PI/180;

  float klong=1;

  float klat=0.6;

  
  //Serial.printf("acc X %f\n",stability_info.accel_x);
  // stability_info.accel_x;
  // stability_info.gyro_x;
  // stability_info.mag_x;
  // stability_info.pitch;
  // stability_info.roll;

  angular_speed_rear_wheels.omega_right = module*(klong*MAX_CURRENT*sin(degree*degtorad))-module*klat*MAX_CURRENT*cos(degree*degtorad);

  angular_speed_rear_wheels.omega_left = module*(klong*MAX_CURRENT*sin(degree*degtorad))+module*klat*MAX_CURRENT*cos(degree*degtorad);


  #if DEBUG_LEVEL > 1
    Serial.printf("CURRENT_LX -> %f\t CURRENT_RX -> %f\n",angular_speed_rear_wheels.omega_left,angular_speed_rear_wheels.omega_right);
  #endif


  reference_CURRENT_rear_wheels.CURRENT_lx = angular_speed_rear_wheels.omega_left;
  reference_CURRENT_rear_wheels.CURRENT_rx = angular_speed_rear_wheels.omega_right;
}

void ECU::set_level_speed_profile(int level){
  switch (level)
  {
  case 1:
    MAX_CURRENT = 15.0;
    break;

  case 2:
    MAX_CURRENT = 30.0;
    break;
  
  case 3:
    MAX_CURRENT = 45.0;
    break;

  case 4:
    MAX_CURRENT = 60.0;
    break;

  case 5:
    MAX_CURRENT = 75.0;
    break;

  case 6:
    MAX_CURRENT = 80.0;
    break;

  default:
    Serial.printf("Error: speed profile not recognised");
    break;
  }
}