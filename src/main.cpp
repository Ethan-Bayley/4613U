#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cstddef>

class TogglePneumatic {
private:
  pros::adi::DigitalOut solenoid;
  bool state = false;

public:
  TogglePneumatic(char port) : solenoid(port) {}

  void toggle() {
    state = !state;
    solenoid.set_value(state);
  }

  void set(bool value) {
    state = value;
    solenoid.set_value(state);
  }

  bool get() const { return state; }
};

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({10, 9, -8});  // Creates a motor group with forwards
pros::MotorGroup right_mg({-2, -3, 4}); // Creates a motor group with forwards

pros::Motor intake_hook(5);           // Creates a motor with port 4
pros::Motor intake_preroller(-6);     // Creates a motor with port 5
pros::Motor lady_brown_motor(-7);      // Creates a motor with port 6
pros::Rotation lady_brown_sensor(18); // Creates a rotation sensor with port 7
pros::Optical
    colour_sort_optical_sensor(19); // Creates an optical sensor with port 8
pros::Distance
    colour_sort_distance_sensor(11); // Creates a distance sensor with port 9

// ODOM

pros::Imu imu(14);

pros::Rotation left_vertical_sensor(-17);

pros::Rotation right_vertical_sensor(12);

// Pneumatics
TogglePneumatic clamp('G');
TogglePneumatic left_doinker('H');
TogglePneumatic right_doinker('F');

// PID

// lateral PID controller

double lateral_kP = 10;
double lateral_kI = 0;
double lateral_kD = 3;

// angular PID controller

double angular_kP = 10;
double angular_kI = 0;
double angular_kD = 3;

lemlib::TrackingWheel left_vertical_tracking_wheel(&left_vertical_sensor,
                                                   lemlib::Omniwheel::NEW_275,
                                                   3.541775);

lemlib::TrackingWheel right_vertical_tracking_wheel(&right_vertical_sensor,
                                                    lemlib::Omniwheel::NEW_275,
                                                    -3.541775);

// ODOM Inisialization

lemlib::OdomSensors sensors(&left_vertical_tracking_wheel,
                            &right_vertical_tracking_wheel, nullptr, nullptr,
                            &imu);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(lateral_kP, // proportional gain (kP)
                       lateral_kI, // integral gain (kI)
                       lateral_kD, // derivative gain (kD)
                       3,          // anti windup
                       1,          // small error range, in inches
                       100,        // small error range timeout, in milliseconds
                       3,          // large error range, in inches
                       500,        // large error range timeout, in milliseconds
                       20          // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(angular_kP, // proportional gain (kP)
                       angular_kI, // integral gain (kI)
                       angular_kD, // derivative gain (kD)
                       3,          // anti windup
                       1,          // small error range, in degrees
                       100,        // small error range timeout, in milliseconds
                       3,          // large error range, in degrees
                       500,        // large error range timeout, in milliseconds
                       0           // maximum acceleration (slew)
    );

lemlib::Drivetrain drivetrain(&left_mg, &right_mg, 10.72599,
                              lemlib::Omniwheel::NEW_325, 450, 2);

lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

enum class Team { NONE, RED, BLUE };
Team selected_team = Team::NONE;

enum auton { POSITIVE, NEGATIVE, SOLO_AWP, NONE };
enum auton selected_auton = NONE;

bool auton_selection_open = true;

std::vector<lv_obj_t *> buttons;

lv_obj_t *label;
std::string team = "Red";
std::string touchLadder = "Ladder";
std::string allianceStake = "Early";
std::string wallStake = "Wall Stake";
std::string side = "Negative";
std::string soloAWP = "Solo Awp";
std::string runAuto = "Run";

#define button_row_padding 10
#define button_row_width 240
#define button_row_height (240 - 7 * button_row_padding) / 6

void updateLabel() {
  std::string text = "Team: " + team + "\n" + "Ladder: " + touchLadder + "\n" +
                     "Alliance Stake: " + allianceStake + "\n" +
                     "Wall Stake: " + wallStake + "\n" + "Side: " + side +
                     "\n" + "Solo Awp: " + soloAWP + "\n" +
                     "Run Auto: " + runAuto;
  lv_label_set_text(label, text.c_str());
}

lv_obj_t *makeButton(lv_obj_t *parent, const char *txt,
                     std::string selected_text, lv_event_cb_t cb,
                     uint32_t bg_color, int row, int index, int buttons_in_row,
                     int column) {
  lv_obj_t *btn = lv_btn_create(parent);
  lv_style_t *style = new lv_style_t;
  lv_style_init(style);

  bool selected = false;

  // Check if this button is selected
  if (selected_text == txt) {
    selected = true;
  } else {
    selected = false;
  }

  // Dim background if selected
  uint32_t final_bg_color = bg_color;
  uint32_t final_border_color = 0x000000;

  if (selected) {
    // Extract RGB from bg_color
    int r = (bg_color >> 16) & 0xFF;
    int g = (bg_color >> 8) & 0xFF;
    int b = bg_color & 0xFF;

    // Darken each component
    r = static_cast<int>(r * 0.8);
    g = static_cast<int>(g * 0.8);
    b = static_cast<int>(b * 0.8);

    // Combine back to 24-bit color
    final_bg_color = (r << 16) | (g << 8) | b;

    // Optional debug print
    std::stringstream ss;
    ss << "0x" << std::hex << std::setw(6) << std::setfill('0')
       << final_bg_color;
    std::cout << "Dimmed color: " << ss.str() << "\n";

    final_border_color = 0xFFFF00;
  }

  // Apply styles
  lv_style_set_bg_color(style, lv_color_hex(final_bg_color));
  lv_style_set_border_color(style, lv_color_hex(final_border_color));
  lv_style_set_border_width(style, 4);
  if (column == 0) {
    lv_obj_align(
        btn, LV_ALIGN_TOP_LEFT,
        button_row_padding + index *
                                 (button_row_width - button_row_padding / 2) /
                                 buttons_in_row,
        row * (button_row_height + button_row_padding) + button_row_padding);
  } else {
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT,
                 button_row_padding +
                     index * (button_row_width - button_row_padding / 2) /
                         buttons_in_row +
                     240 - 5,
                 row * (button_row_height + button_row_padding) +
                     button_row_padding);
  }

  lv_obj_set_size(
      btn,
      (button_row_width - (0.5 + buttons_in_row) * button_row_padding) /
          buttons_in_row,
      button_row_height);

  lv_obj_t *btn_label = lv_label_create(btn);
  lv_label_set_text(btn_label, txt);
  lv_obj_center(btn_label);

  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, (void *)txt);
  lv_obj_add_style(btn, style, 0);

  buttons.push_back(btn);
  return btn;
}

void autonSelectorInit();

void show_auton_summary();

void back_to_auton_menu(lv_event_t *e) {
  auton_selection_open = true;
  lv_obj_clean(lv_scr_act()); // Clear the screen
  autonSelectorInit();        // Relaunch the auton GUI
}

void show_auton_summary() {
  auton_selection_open = false;
  lv_obj_clean(lv_scr_act());

  // Back Button to reopen auton menu
  lv_obj_t *back_btn = lv_btn_create(lv_scr_act());
  lv_obj_align(back_btn, LV_ALIGN_TOP_LEFT, 10, 10);
  lv_obj_set_size(back_btn, 80, 40);

  lv_obj_t *back_label = lv_label_create(back_btn);
  lv_label_set_text(back_label, "Back");

  lv_obj_add_event_cb(back_btn, back_to_auton_menu, LV_EVENT_CLICKED, nullptr);

  // Display selected auton settings
  std::string summary_text = "Team: " + team + "\n" + "Ladder: " + touchLadder +
                             "\n" + "Alliance Stake: " + allianceStake + "\n" +
                             "Wall Stake: " + wallStake + "\n" +
                             "Side: " + side + "\n" + "Solo Awp: " + soloAWP +
                             "\n" + "Run Auto: " + runAuto;

  lv_obj_t *summary_label = lv_label_create(lv_scr_act());
  lv_label_set_text(summary_label, summary_text.c_str());
  lv_obj_align(summary_label, LV_ALIGN_TOP_LEFT, 10, 60);

  // Display robot position

  pros::Task showPosition([&] {
    if (!auton_selection_open) {
      std::string position_info =
          "X: " + std::to_string(chassis.getPose().x) + " in\n" +
          "Y: " + std::to_string(chassis.getPose().y) + " in\n" +
          "Heading: " + std::to_string(chassis.getPose().theta) + "°";

      lv_obj_t *pos_label = lv_label_create(lv_scr_act());
      lv_label_set_text(pos_label, position_info.c_str());
      lv_obj_align(pos_label, LV_ALIGN_TOP_LEFT, 10, 180);
    } else {
    }

    pros::delay(100);
  });
  selected_team = (team == "Red") ? Team::RED : Team::BLUE;
}

void button_event_cb(lv_event_t *e) {
  const char *choice = (const char *)lv_event_get_user_data(e);
  lv_obj_t *btn = lv_event_get_target(e);

  std::string label_txt = lv_label_get_text(lv_obj_get_child(btn, 0));

  std::string str = label_txt;
  if (str == "Red" || str == "Blue")
    team = str;
  else if (str == "Ladder" || str == "No Ladder")
    touchLadder = str;
  else if (str == "Early" || str == "Delayed" || str == "None")
    allianceStake = str;
  else if (str == "Wall Stake" || str == "No Wall Stake") {
    wallStake = str;
  } else if (str == "Negative" || str == "Positive") {
    side = str;
  } else if (str == "Solo Awp" || str == "No Solo Awp") {
    soloAWP = str;
  } else if (str == "Run" || str == "Only L" || str == "Off Line" ||
             str == "No") {
    runAuto = str;
  } else if (str == "Done") {
    show_auton_summary(); // Show summary screen
    return;               // Exit the selector early
  }

  lv_obj_clean(lv_scr_act());
  autonSelectorInit();
}

void autonSelectorInit() {
  lv_obj_t *scr = lv_scr_act();

  label = lv_label_create(scr);
  lv_obj_align(label, LV_ALIGN_TOP_RIGHT, -10, 10);
  updateLabel();
  int row = 0;
  int index = 0;
  int buttons_in_row = 2;
  int column = 0;
  // TEAM
  makeButton(scr, "Red", team, button_event_cb, 0xFF0000, row, index,
             buttons_in_row, column);
  index++;
  makeButton(scr, "Blue", team, button_event_cb, 0x0000FF, row, index,
             buttons_in_row, column);
  index++;

  // TOUCH LADDER
  index = 0;
  row++;
  makeButton(scr, "Ladder", touchLadder, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);
  index++;
  makeButton(scr, "No Ladder", touchLadder, button_event_cb, 0x424242, row,
             index, buttons_in_row, column);
  index++;

  // ALLIANCE STAKE
  index = 0;
  row++;
  buttons_in_row = 3;
  makeButton(scr, "Early", allianceStake, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);
  index++;
  makeButton(scr, "Delayed", allianceStake, button_event_cb, 0x424242, row,
             index, buttons_in_row, column);
  index++;
  makeButton(scr, "None", allianceStake, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);
  index++;

  // WALL STAKE
  buttons_in_row = 2;
  index = 0;
  row++;
  makeButton(scr, "Wall Stake", wallStake, button_event_cb, 0x424242, row,
             index, buttons_in_row, column);
  index++;
  makeButton(scr, "No Wall Stake", wallStake, button_event_cb, 0x424242, row,
             index, buttons_in_row, column);
  index++;

  // SIDE
  index = 0;
  row++;
  makeButton(scr, "Negative", side, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);
  index++;
  makeButton(scr, "Positive", side, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);
  index++;

  // SOLO AWP
  index = 0;
  row++;
  makeButton(scr, "Solo Awp", soloAWP, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);
  index++;
  makeButton(scr, "No Solo Awp", soloAWP, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);

  column++;
  index = 0;
  row = 0;
  buttons_in_row = 4;

  makeButton(scr, "Run", runAuto, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);
  index++;
  makeButton(scr, "Only L", runAuto, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);
  index++;
  makeButton(scr, "Off Line", runAuto, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);
  index++;
  makeButton(scr, "No", runAuto, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);

  index = 0;
  row = 5;
  buttons_in_row = 1;

  makeButton(scr, "Done", runAuto, button_event_cb, 0x424242, row, index,
             buttons_in_row, column);
}

double apply_input_scale(double number) {
  double abs_number = std::abs(number);
  double direction = (number == 0) ? 0 : (number / abs_number);
  if (abs_number < 0.1) {
    abs_number = 0;
  } else if (abs_number < 0.626628) {
    abs_number = pow(abs_number + 0.1, 3) - 0.1;
  } else if (abs_number <= 1.0) {
    abs_number = pow(abs_number, 3);
  }
  abs_number = abs_number * direction;
  return abs_number;
}
class LiftPosition {
public:
  enum LiftPositionEnum {
    DOWN = 0 * 100,
    LOAD = 36 * 100,
    HALF_WAY = 90 * 100,
    WALL_STAKE = 150 * 100,
    ALLIANCE_STAKE = 180 * 100
  };
};

const int num_states = 3;
int states[num_states] = {LiftPosition::DOWN, 
                          LiftPosition::LOAD,
                          LiftPosition::WALL_STAKE

};

int curr_state = 0;
int target = lady_brown_motor.get_position();

void next_state() {
  curr_state += 1;
  if (curr_state == num_states) {
    curr_state = 0;
  }
  target = states[curr_state];
}

void lift_control() {
  double kP = 0.03;
  double error = target - lady_brown_sensor.get_position();
  double velocity = kP * error;
  lady_brown_motor.move(velocity);
}

bool color_sort_enabled = true;

void colour_sort() {
  pros::c::optical_rgb_s_t colour =
      colour_sort_optical_sensor.get_rgb(); // Get the RGB values
  int distance = colour_sort_distance_sensor.get_distance(); // Get the distance
  if (color_sort_enabled) {
    if (distance < 70) { // If the distance is less than 70
      if (team == "Red") {
        if (colour.blue > colour.red && colour.blue > colour.green) {
          intake_hook.move_velocity(0);
          pros::delay(1000);
          intake_hook.move_velocity(100);
        }
      } else if (team == "Blue") {
        if (colour.red > colour.green && colour.red > colour.blue) {
          intake_hook.move_velocity(0);
          pros::delay(1000);
          intake_hook.move_velocity(100);
        }
      }
    }
  }
}

void home_motor(pros::Motor &motor, pros::Rotation &sensor) {
  motor.move(1);
  while (motor.get_actual_velocity() > 0) {
    pros::delay(10);
  }
  motor.move(0);
  sensor.reset();
}

int selected_auton_index = 0;

void initialize() {
  pros::lcd::initialize();
  /*
  pros::c::lcd_initialize();
  lvgl_init();
  autonSelectorInit();


  while (auton_selection_open) {
    pros::delay(10);
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      break;
    }
  }
    */
    lady_brown_sensor.set_position(0);
  chassis.setPose(0, 0, 0);
}

void disabled() {}

void competition_initialize() {}

class global_utils {
public:
  void wait_for_rings_pass_intake(int num_rings, void (*callback)()) {
    int rings = 0;
    while (rings < num_rings) {
      pros::c::optical_rgb_s_t colour = colour_sort_optical_sensor.get_rgb();
      if (colour_sort_distance_sensor.get_distance() < 70 &&
              (colour.blue > colour.red && colour.blue > colour.green) ||
          (colour.red > colour.green && colour.red > colour.blue)) {
        rings++;
      } else {
        pros::delay(10);
      }
    }

    pros::delay(100);
    if (callback)
      callback();
  }

  void check_ladybrown_load(int target_position) {
    bool is_loaded = false;
    while (!is_loaded) {
      if (intake_hook.get_voltage() > 1 &&
          intake_hook.get_actual_velocity() < 1) {
        intake_hook.move(0);
        pros::delay(100);
        target = target_position;
        is_loaded = true;
      } else {
        pros::delay(10);
      }
    }
  }

  static void clamp_toggle_callback() { clamp.toggle(); }
};

class autonomous_sections_Red {
private:
public:
  void goal_rush() {
    chassis.moveToPose(-1.375 * 23.3, -50, 26.5650512, 0);
    right_doinker.toggle();
    chassis.moveToPose(1.75 * 23.3, -0.5 * 23.3, 26.5650512, 0);
    right_doinker.toggle();
    chassis.moveToPose(1.5 * 23.3, -1 * 23.3, 26.5650512, 5);
    right_doinker.toggle();
    chassis.moveToPose(1 * 23.3, -1 * 23.3, 150, 5);
    right_doinker.toggle();
  }

  void ring_rish() {
    intake_preroller.move(127);
    chassis.moveToPose(-1.75 * 23.3, -0.5 * 23.3, 360 - 26.5650512, 0);
    left_doinker.toggle();
    chassis.moveToPose(-1.5 * 23.3, -0.5 * 23.3, 360 - 30, 0);
    chassis.moveToPose(-1 * 23.3, -1 * 23.3, 360 - 30, 0);
    left_doinker.toggle();
  }

  void alliance_stake(int side) {
    if (side == POSITIVE) {
      chassis.moveToPose(7 * sqrt(2), -3 * 23.3 + 7 * sqrt(2), 225, 0);
      target = (LiftPosition::ALLIANCE_STAKE);
      pros::delay(400);
      curr_state = num_states;
      next_state();

    } else if (side == NEGATIVE) {
      chassis.moveToPose(-7 * sqrt(2), -3 * 23.3 + 7 * sqrt(2), 135, 0);
      target = (LiftPosition::ALLIANCE_STAKE);
      pros::delay(400);
      curr_state = num_states;
      next_state();
    }
  }

  void wall_stake(int side) {
    if (side == POSITIVE) {
      chassis.moveToPose(-2.5 * 23.3, -0.5 * 23.3, 45, 0);
      intake_hook.move(0);
      next_state();
      chassis.moveToPose(-2.75 * 23.3, -0.25 * 23.3, 45, 0);

    } else if (side == NEGATIVE) {
      chassis.moveToPose(2.5 * 23.3, -0.5 * 23.3, 360 - 45, 0);
      intake_hook.move(0);
      next_state();
      chassis.moveToPose(2.75 * 23.3, -0.25 * 23.3, 45, 0);
    }
  }

  void clear_corner_with_doinker(int side, int varient) {
    if (side == POSITIVE && varient == 0) {
      chassis.moveToPose(2 * 23.3, -2.5 * 23.3, 90, 0);
      right_doinker.toggle();
      chassis.moveToPose(2.5 * 23.3, -2.5 * 23.3, 90, 0);
      chassis.moveToPose(2.5 * 23.3, -2.5 * 23.3, 45, 0);
      right_doinker.toggle();
    } else if (side == POSITIVE && varient == 1) {
      chassis.moveToPose(2.5 * 23.3, -2 * 23.3, 180, 0);
      left_doinker.toggle();
      chassis.moveToPose(2.5 * 23.3, -2.5 * 23.3, 180, 0);
      chassis.moveToPose(2.5 * 23.3, -2.5 * 23.3, 180 + 45, 0);
      left_doinker.toggle();
    } else if (side == NEGATIVE && varient == 0) {
      chassis.moveToPose(-2 * 23.3, -2.5 * 23.3, 270, 0);
      left_doinker.toggle();
      chassis.moveToPose(-2.5 * 23.3, -2.5 * 23.3, 270, 0);
      chassis.moveToPose(-2.5 * 23.3, -2.5 * 23.3, 315, 0);
      left_doinker.toggle();
    } else if (side == NEGATIVE && varient == 1) {
      chassis.moveToPose(-2.5 * 23.3, -2 * 23.3, 180, 0);
      right_doinker.toggle();
      chassis.moveToPose(-2.5 * 23.3, -2.5 * 23.3, 180, 0);
      chassis.moveToPose(-2.5 * 23.3, -2.5 * 23.3, 135, 0);
      right_doinker.toggle();
    }
  }

  void clear_corner_with_intake(int side, int varient) {
    if (side == POSITIVE && varient == 0) {
      intake_hook.move(127);
      intake_preroller.move(127);
      chassis.moveToPose(2.5 * 23.3, -2.5 * 23.3, 135, 0);
      chassis.moveToPose(2.75 * 23.3, -2.75 * 23.3, 135, 0);
      chassis.moveToPose(2.5 * 23.3, -2.5 * 23.3, 135, 0);
      chassis.moveToPose(2.75 * 23.3, -2.75 * 23.3, 135, 0);
      chassis.moveToPose(2.5 * 23.3, -2.5 * 23.3, 135, 0);
      chassis.moveToPose(2.75 * 23.3, -2.75 * 23.3, 135, 0);
    } else if (side == POSITIVE && varient == 1) {
      intake_hook.move(127);
      intake_preroller.move(127);
      chassis.moveToPose(2.75 * 23.3, -2.75 * 23.3, 135, 0);
    } else if (side == NEGATIVE && varient == 0) {
      intake_hook.move(127);
      intake_preroller.move(127);
      chassis.moveToPose(-2.5 * 23.3, -2.5 * 23.3, 225, 0);
      chassis.moveToPose(-2.75 * 23.3, -2.75 * 23.3, 225, 0);
      chassis.moveToPose(-2.5 * 23.3, -2.5 * 23.3, 225, 0);
      chassis.moveToPose(-2.75 * 23.3, -2.75 * 23.3, 225, 0);
      chassis.moveToPose(-2.5 * 23.3, -2.5 * 23.3, 225, 0);
      chassis.moveToPose(-2.75 * 23.3, -2.75 * 23.3, 225, 0);
    } else if (side == NEGATIVE && varient == 1) {
      intake_hook.move(127);
      intake_preroller.move(127);
      chassis.moveToPose(-2.5 * 23.3, -2.5 * 23.3, 225, 0);
      chassis.moveToPose(-2.75 * 23.3, -2.75 * 23.3, 225, 0);
    }
  }

  void get_centre_rings(int side, int varient) {
    if (side == POSITIVE && varient == 0) {
      chassis.moveToPose(1 * 23.3, -1 * 23.3, 315, 0);
      chassis.moveToPose(0.5 * 23.3, -0.25 * 23.3, 270, 0);
      chassis.moveToPose(0.25 * 23.3, -0.25 * 23.3, 315, 0);
      left_doinker.toggle();
      chassis.moveToPose(0.25 * 23.3, -0.25 * 23.3, 340, 0);
      right_doinker.toggle();
    } else if (side == POSITIVE && varient == 1) {
      chassis.moveToPose(1 * 23.3, -1 * 23.3, 45, 0);
      intake_preroller.move(127);
      intake_hook.move(127);
      chassis.moveToPose(2 * 23.3, -7.444, 90, 0);
    } else if (side == NEGATIVE && varient == 0) {
      chassis.moveToPose(-1 * 23.3, -1 * 23.3, 45, 0);
      chassis.moveToPose(-0.5 * 23.3, -0.25 * 23.3, 90, 0);
      chassis.moveToPose(-0.25 * 23.3, -0.25 * 23.3, 45, 0);
      right_doinker.toggle();
      chassis.moveToPose(-0.25 * 23.3, -0.25 * 23.3, 20, 0);
      left_doinker.toggle();
    } else if (side == NEGATIVE && varient == 1) {
      chassis.moveToPose(-1 * 23.3, -1 * 23.3, 315, 0);
      intake_preroller.move(127);
      intake_hook.move(127);
      chassis.moveToPose(-2 * 23.3, -7.444, 270, 0);
    }
  }

  void touch_ladder() {
    target = (LiftPosition::HALF_WAY);
    chassis.moveToPose(0.8 * 23.3, -0.8 * 23.3, 315, 0);
    target = (LiftPosition::LOAD);
  }
};

void autonomous() {
  global_utils utils;
  if (runAuto == "Run") {

    if (selected_team == Team::RED) {
      autonomous_sections_Red red_auton;
      if (soloAWP == "Solo Awp") {
        if (side == "Positive") {
          chassis.setPose(-50, 23.3, 180);

          pros::Task wait_for_rings_task_1([&] {
            utils.wait_for_rings_pass_intake(3, next_state);
            wait_for_rings_task_1.remove();
          });

          chassis.moveToPose(-1 * 23.3, -1 * 23.3, 180, 0);
          clamp.toggle();
          wait_for_rings_task_1.remove();
          pros::Task check_ladybrown_load_task([&] {
            utils.check_ladybrown_load((LiftPosition::HALF_WAY));
            check_ladybrown_load_task.remove();
          });

          red_auton.get_centre_rings(POSITIVE, 1);
          chassis.moveToPose(-1.25 * 23.3, 15.372, 270, 0);
          chassis.moveToPose(-0.5 * 23.3, -2 * 23.3, 90, 0);
          chassis.moveToPose(1 * 23.3, -2 * 23.3, 90, 0);
          check_ladybrown_load_task.remove();
          pros::Task wait_for_rings_task_2([&] {
            utils.wait_for_rings_pass_intake(2, utils.clamp_toggle_callback);
            wait_for_rings_task_2.remove();
          });

          red_auton.alliance_stake(POSITIVE);
          if (pros::Task::current().get_name() ==
              wait_for_rings_task_2.get_name()) {
            wait_for_rings_task_2.remove();

            chassis.moveToPose(0, -2 * 23.3, 90, 0);
            clamp.toggle();
          }

          chassis.moveToPose(1 * 23.3, -1 * 23.3, 210, 0);
          clamp.toggle();
          intake_hook.move(127);
          intake_preroller.move(127);
          chassis.moveToPose(2 * 23.3, -1 * 23.3, 90, 0);
          pros::Task lady_brown_position([&] {
            pros::delay(400);
            target = (LiftPosition::HALF_WAY);
          });
          chassis.moveToPose(0.8 * 23.3, -0.8 * 23.3, 315, 0);

        } else if (side == "Negative") {
        }
      } else if (side == "Positive") {
        if (allianceStake == "Early") {

          chassis.setPose(7 * sqrt(2), -3 * 23.3 + 7 * sqrt(2), 225);
          red_auton.alliance_stake(POSITIVE);
        } else {
          chassis.setPose(-1.375 * 23.3, -50, 26.5650512);
        }
        red_auton.goal_rush();
        clamp.toggle();
        red_auton.get_centre_rings(POSITIVE, 0);
        chassis.moveToPose(1 * 23.3, -1 * 23.3, 315, 0);
        intake_preroller.move(127);
        intake_hook.move(127);
        chassis.moveToPose(0.5 * 23.3, -0.5 * 23.3, 315, 0);
        clamp.toggle();
        chassis.moveToPose(2.5 * 23.3, -0.5 * 23.3, 180 + 26.5650512, 0);
        clamp.toggle();
        if (wallStake == "Wall Stake") {
          next_state();
          pros::Task check_ladybrown_load_task([&] {
            utils.check_ladybrown_load((LiftPosition::HALF_WAY));
            check_ladybrown_load_task.remove();
          });
          red_auton.wall_stake(POSITIVE);
        } else if (wallStake == "No Wall Stake") {
          pros::Task wait_for_rings_task_1([&] {
            utils.wait_for_rings_pass_intake(2, next_state);
            wait_for_rings_task_1.remove();
          });
        }

        chassis.moveToPose(2 * 23.3, -1 * 23.3, 135, 0);
        red_auton.clear_corner_with_intake(POSITIVE, 0);

        if (touchLadder == "Ladder") {
          red_auton.touch_ladder();
        } else if (touchLadder == "No Ladder") {
          if (wallStake == "Wall Stake") {
            red_auton.clear_corner_with_intake(POSITIVE, 0);
            red_auton.clear_corner_with_doinker(POSITIVE, 0);
          } else if (wallStake == "No Wall Stake") {
            red_auton.clear_corner_with_doinker(POSITIVE, 0);
          }
          chassis.moveToPose(2.5 * 23.3, -2.5 * 23.3, 135, 0);
          chassis.moveToPose(2.75 * 23.3, -2.75 * 23.3, 135, 0);
        }
      }
    }
  } else if (runAuto == "Ladder") {
    pros::delay(12000);
    if ((selected_team == Team::RED && side == "Positive") ||
        (selected_team == Team::BLUE && side == "Negative")) {
      chassis.setPose(1 * 23.3, -2.5 * 23.3, 0);
      target = (LiftPosition::HALF_WAY);
      chassis.moveToPose(0.2 * 23.3, -1.2 * 23.3, 315, 0);
      target = (LiftPosition::ALLIANCE_STAKE);
    } else if ((selected_team == Team::RED && side == "Negitive") ||
               (selected_team == Team::BLUE && side == "Positive")) {
      chassis.setPose(-1 * 23.3, -2.5 * 23.3, 0);
      target = (LiftPosition::HALF_WAY);
      chassis.moveToPose(-0.2 * 23.3, -1.2 * 23.3, 315, 0);
      target = (LiftPosition::ALLIANCE_STAKE);
    }
  } else if (runAuto == "Off Line") {
    pros::delay(12000);
    chassis.setPose(0 * 23.3, 0 * 23.3, 0);
    chassis.moveToPose(0 * 23.3, 1 * 23.3, 0, 0);
  } else if (runAuto == "None") {
    // Do nothing
  } else {
    chassis.setPose(0 * 23.3, 0 * 23.3, 0);
    chassis.moveToPose(0 * 23.3, 1 * 23.3, 0, 0);
  }
}

bool pressed = false;

void opcontrol() {
  pros::Task lift_control_task([&]() {
    while (true) {
      lift_control();
      pros::delay(20);
    }
  });

  pros::Task colour_sort_task([&]() {
    while (true) {
      colour_sort();
      pros::delay(20);
    }
  });

  pressed = false;
  // Creates a digital out with port D
  while (true) {

    // Arcade control scheme
    int dir = master.get_analog(
        ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
    int turn = master.get_analog(
        ANALOG_RIGHT_X); // Gets the turn left/right from right joystick

    dir = apply_input_scale(
        dir); // Applies input scale to the forward/backward value
    turn = apply_input_scale(turn); // Applies input scale to the turn value

    chassis.arcade(dir, turn);

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
      clamp.toggle();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
      next_state();
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake_hook.move(-127);
      intake_preroller.move(127);
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake_hook.move(127);
      intake_preroller.move(-127);
    } else {
      intake_hook.move(0);
      intake_preroller.move(0);
    }

    if ((master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) && pressed == false) {
      lift_control_task.suspend();
      pressed = true;
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        lady_brown_motor.move_velocity(127);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        lady_brown_motor.move_velocity(-127);
    }

    if (!(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) && pressed == true) {
        target = lady_brown_sensor.get_position();
        lift_control_task.resume();
        pressed = false;
      }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
      left_doinker.toggle();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      right_doinker.toggle();
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      curr_state = num_states;
      next_state();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      chassis.setPose(0, 0, 0);
      chassis.moveToPose(0, 24, 0, 1000000000);
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      chassis.setPose(0, 0, 0);
      chassis.moveToPose(0, 0, 90, 1000000000);
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      color_sort_enabled = !color_sort_enabled;
    }

    pros::delay(20); // Run for 20 ms then update
  }
}