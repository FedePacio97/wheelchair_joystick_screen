#include "Screen.h"
#include "ECU.h"

uint32_t Screen::current_speed_profile = 1;

NexNumber Screen::profile_text = NexNumber(0,15,"n1");
NexPicture Screen::profile_img = NexPicture(0,16,"p8");

Screen::Screen(){
  
}

void Screen::listen_touch_event(){
  nexLoop(nex_listen_list);
}

void Screen::adjust_speed_profile(int increment){
  if( (current_speed_profile + increment > MAX_SPEED_PROFILE) || (current_speed_profile + increment < MIN_SPEED_PROFILE) )
    return;

  current_speed_profile += increment;

  //Update text label
  profile_text.setValue(current_speed_profile);

  //Update ECU "MAX_RPM"
  ECU::set_level_speed_profile(current_speed_profile);

  //Update gauge image
  switch (current_speed_profile)
  {
    case 1:
      profile_img.setPic(SPEED_PROFILE_1_IMG_ID);
      break;

    case 2:
      profile_img.setPic(SPEED_PROFILE_2_IMG_ID);
      break;

    case 3:
      profile_img.setPic(SPEED_PROFILE_3_IMG_ID);
      break;

    case 4:
      profile_img.setPic(SPEED_PROFILE_4_IMG_ID);
      break;

    case 5:
      profile_img.setPic(SPEED_PROFILE_5_IMG_ID);
      break;

    case 6:
      profile_img.setPic(SPEED_PROFILE_6_IMG_ID);
      break;

    default:
      break;
  }


}

void p0_profile_up_release(void *ptr) {
  Screen::adjust_speed_profile(1);
}

void p0_profile_down_release(void *ptr) {
  Screen::adjust_speed_profile(-1);
}

void Screen::set_battery_level(int level){
  String level_battery = String(level) + '%';

  battery_level_text.setText(level_battery.c_str());
  switch (level)
  {
    case 0 ... 25:
      battery_level_image.setPic(LOW_BATTERY_IMG_ID);
      break;
    
    case 26 ... 50:
      battery_level_image.setPic(MEDIUM_BATTERY_IMG_ID);
      break;

    case 51 ... 75:
      battery_level_image.setPic(HIGH_BATTERY_IMG_ID);
      break;

    default:
      battery_level_image.setPic(FULL_BATTERY_IMG_ID);
      break;

  }
}

void Screen::set_roll(int angle){
  roll_text.setValue(abs(angle));
  switch (abs(angle))
  {
    case 0 ... 10:
      /*if(angle < 0)
          roll_image.setPic(ROLL_0_10_NEGATIVE_IMG_ID);
      else
          roll_image.setPic(ROLL_0_10_POSITIVE_IMG_ID);*/

      break;

    case 11 ... 20:
      /*if(angle < 0)
          roll_image.setPic(ROLL_11_20_NEGATIVE_IMG_ID);
      else
          roll_image.setPic(ROLL_11_20_POSITIVE_IMG_ID);*/
      break;

    case 21 ... 30:
      /*if(angle < 0)
          roll_image.setPic(ROLL_21_30_NEGATIVE_IMG_ID);
      else
          roll_image.setPic(ROLL_21_30_POSITIVE_IMG_ID);*/
      break;

    case 31 ... 40:
      /*if(angle < 0)
          roll_image.setPic(ROLL_31_40_NEGATIVE_IMG_ID);
      else
          roll_image.setPic(ROLL_31_40_POSITIVE_IMG_ID);*/
      break;

    case 41 ... 50:
      /*if(angle < 0)
          roll_image.setPic(ROLL_41_50_NEGATIVE_IMG_ID);
      else
          roll_image.setPic(ROLL_41_50_POSITIVE_IMG_ID);*/
      break;

    case 51 ... 60:
      /*if(angle < 0)
          roll_image.setPic(ROLL_51_60_NEGATIVE_IMG_ID);
      else
          roll_image.setPic(ROLL_51_60_POSITIVE_IMG_ID);*/
      break;

    case 61 ... 70:
      /*if(angle < 0)
          roll_image.setPic(ROLL_61_70_NEGATIVE_IMG_ID);
      else
          roll_image.setPic(ROLL_61_70_POSITIVE_IMG_ID);*/
      break;

    case 71 ... 80:
      /*if(angle < 0)
          roll_image.setPic(ROLL_71_80_NEGATIVE_IMG_ID);
      else
          roll_image.setPic(ROLL_71_80_POSITIVE_IMG_ID);*/
      break;

    case 81 ... 90:
      /*if(angle < 0)
          roll_image.setPic(ROLL_81_90_NEGATIVE_IMG_ID);
      else
          roll_image.setPic(ROLL_81_90_POSITIVE_IMG_ID);*/
      break;

    default:
      break;

  }
}

void Screen::set_pitch(int angle){

  pitch_text.setValue(abs(angle));
  switch (abs(angle))
  {
  case 0 ... 10:
    /*if(angle < 0)
      pitch_image.setPic(PITCH_0_10_NEGATIVE_IMG_ID);
    else
      pitch_image.setPic(PITCH_0_10_POSITIVE_IMG_ID);*/

    break;
  
  case 11 ... 20:
    /*if(angle < 0)
      pitch_image.setPic(PITCH_11_20_NEGATIVE_IMG_ID);
    else
      pitch_image.setPic(PITCH_11_20_POSITIVE_IMG_ID);*/
    break;

  case 21 ... 30:
    /*if(angle < 0)
      pitch_image.setPic(PITCH_21_30_NEGATIVE_IMG_ID);
    else
      pitch_image.setPic(PITCH_21_30_POSITIVE_IMG_ID);*/
    break;
  
  case 31 ... 40:
    /*if(angle < 0)
      pitch_image.setPic(PITCH_31_40_NEGATIVE_IMG_ID);
    else
      pitch_image.setPic(PITCH_31_40_POSITIVE_IMG_ID);*/
    break;

  case 41 ... 50:
    /*if(angle < 0)
      pitch_image.setPic(PITCH_41_50_NEGATIVE_IMG_ID);
    else
      pitch_image.setPic(PITCH_41_50_POSITIVE_IMG_ID);*/
    break;

  case 51 ... 60:
    /*if(angle < 0)
      pitch_image.setPic(PITCH_51_60_NEGATIVE_IMG_ID);
    else
      pitch_image.setPic(PITCH_51_60_POSITIVE_IMG_ID);*/
    break;
  
  case 61 ... 70:
    /*if(angle < 0)
      pitch_image.setPic(PITCH_61_70_NEGATIVE_IMG_ID);
    else
      pitch_image.setPic(PITCH_61_70_POSITIVE_IMG_ID);*/
    break;
  
  case 71 ... 80:
    /*if(angle < 0)
      pitch_image.setPic(PITCH_71_80_NEGATIVE_IMG_ID);
    else
      pitch_image.setPic(PITCH_71_80_POSITIVE_IMG_ID);*/
    break;
  
  case 81 ... 90:
    /*if(angle < 0)
      pitch_image.setPic(PITCH_81_90_NEGATIVE_IMG_ID);
    else
      pitch_image.setPic(PITCH_81_90_POSITIVE_IMG_ID);*/
    break;

  default:
    break;

  }
}

void Screen::set_speed(int speed){
  speed_text.setValue(speed);
}

void Screen::set_km(float km){
  km_text.setText(String(km).c_str());
}

void Screen::attach_listeners(){
  profile_down.attachPop(p0_profile_down_release, &profile_down);
  profile_up.attachPop(p0_profile_up_release, &profile_up);
}

bool Screen::initialize(){
  return nexInit();
}