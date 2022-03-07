#ifndef Screen_h
#define Screen_h

#include "Nextion.h"

#define LOW_BATTERY_IMG_ID 1
#define MEDIUM_BATTERY_IMG_ID 2
#define HIGH_BATTERY_IMG_ID 3
#define FULL_BATTERY_IMG_ID 4

#define ROLL_0_10_POSITIVE_IMG_ID 10  //[0,10]
#define ROLL_0_10_NEGATIVE_IMG_ID 11 //[-10,0]
#define ROLL_11_20_POSITIVE_IMG_ID 10
#define ROLL_11_20_NEGATIVE_IMG_ID 11
#define ROLL_21_30_POSITIVE_IMG_ID 10
#define ROLL_21_30_NEGATIVE_IMG_ID 11
#define ROLL_31_40_POSITIVE_IMG_ID 10
#define ROLL_31_40_NEGATIVE_IMG_ID 11
#define ROLL_41_50_POSITIVE_IMG_ID 10
#define ROLL_41_50_NEGATIVE_IMG_ID 11
#define ROLL_51_60_POSITIVE_IMG_ID 10
#define ROLL_51_60_NEGATIVE_IMG_ID 11
#define ROLL_61_70_POSITIVE_IMG_ID 10
#define ROLL_61_70_NEGATIVE_IMG_ID 11
#define ROLL_71_80_POSITIVE_IMG_ID 10
#define ROLL_71_80_NEGATIVE_IMG_ID 11
#define ROLL_81_90_POSITIVE_IMG_ID 10
#define ROLL_81_90_NEGATIVE_IMG_ID 11

#define PITCH_0_10_POSITIVE_IMG_ID 10  //[0,10]
#define PITCH_0_10_NEGATIVE_IMG_ID 11 //[-10,0]
#define PITCH_11_20_POSITIVE_IMG_ID 10
#define PITCH_11_20_NEGATIVE_IMG_ID 11
#define PITCH_21_30_POSITIVE_IMG_ID 10
#define PITCH_21_30_NEGATIVE_IMG_ID 11
#define PITCH_31_40_POSITIVE_IMG_ID 10
#define PITCH_31_40_NEGATIVE_IMG_ID 11
#define PITCH_41_50_POSITIVE_IMG_ID 10
#define PITCH_41_50_NEGATIVE_IMG_ID 11
#define PITCH_51_60_POSITIVE_IMG_ID 10
#define PITCH_51_60_NEGATIVE_IMG_ID 11
#define PITCH_61_70_POSITIVE_IMG_ID 10
#define PITCH_61_70_NEGATIVE_IMG_ID 11
#define PITCH_71_80_POSITIVE_IMG_ID 10
#define PITCH_71_80_NEGATIVE_IMG_ID 11
#define PITCH_81_90_POSITIVE_IMG_ID 10
#define PITCH_81_90_NEGATIVE_IMG_ID 11

#define SPEED_PROFILE_1_IMG_ID 32
#define SPEED_PROFILE_2_IMG_ID 33
#define SPEED_PROFILE_3_IMG_ID 34
#define SPEED_PROFILE_4_IMG_ID 35
#define SPEED_PROFILE_5_IMG_ID 36
#define SPEED_PROFILE_6_IMG_ID 37

//Build the nex_listen_list that the ESP32 will listen for
//only the buttons contained checked Send Component IDs
/*NexTouch *nex_listen_list[] = {
  &p0_b0, &p0_b1, &p1_b0, NULL
};
*/

#define MAX_SPEED_PROFILE 6
#define MIN_SPEED_PROFILE 1

class Screen{
  //Declare essential MCU side HMI Components
  NexPage p0 = NexPage(0,0,"page0");

  NexPicture battery_level_image = NexPicture(0,2,"p0");
  NexText battery_level_text = NexText(0,8,"t0");

  NexNumber speed_text = NexNumber(0,1,"n0");
  NexText km_text = NexText(0,8,"t1");

  NexNumber pitch_text = NexNumber(0,6,"n3");
  NexPicture pitch_image = NexPicture(0,5,"p1");
  NexNumber roll_text = NexNumber(0,7,"n4");
  NexPicture roll_image = NexPicture(0,8,"p2");

  NexPicture img_fun = NexPicture(0,9,"p3");
  NexPicture img_fun1 = NexPicture(0,10,"p4");

  NexButton profile_down = NexButton(0,17,"b0");
  NexButton profile_up = NexButton(0,18,"b1");

  //They are static for convenience
  //Because we cannot declare p0_profile_up_release,p0_profile_down_release as class member otherwise attachPop complains.
  //So, to allow p0_profile_up_release,p0_profile_down_release to call the method adjust_speed_profile without having a reference to an object, this last one should be static.
  //Since it has been declared static, also the referenced members inside the function should be static
  static NexNumber profile_text;
  static NexPicture profile_img;

  //Build the nex_listen_list that the ESP32 will listen for
  NexTouch *nex_listen_list[3] = {
    &profile_down, &profile_up, NULL
  };


  static uint32_t current_speed_profile;

  public:
    Screen();
    bool initialize();
    static void adjust_speed_profile(int increment);
    void set_battery_level(int level);
    void set_roll(int angle);
    void set_pitch(int angle);
    void set_speed(int speed);
    void set_km(float km);
    void attach_listeners();
    void listen_touch_event();
};


#endif