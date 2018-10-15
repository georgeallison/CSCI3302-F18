#include <sparki.h>
#include <math.h>

#ifndef M_PI
#  define M_PI 3.1415
#endif

#define ROBOT_SPEED 0.0275 // meters/second
#define CYCLE_TIME .050 // Default 50ms cycle time
#define AXLE_DIAMETER 0.0857 // meters
#define WHEEL_RADIUS 0.03 // meters
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_GOTO_POSITION_PART2 2
#define CONTROLLER_GOTO_POSITION_PART3 3

#define FWD 1
#define NONE 0
#define BCK -1


// Line following configuration variables
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

// Controller and dTheta update rule settings
const int current_state = CONTROLLER_GOTO_POSITION_PART3;

//remove
float change_in_rotation = 0;

// Odometry bookkeeping
float orig_dist_to_goal = 0.0;
float pose_x = 0., pose_y = 0., pose_theta = 0.;
float dest_pose_x = 0., dest_pose_y = 0., dest_pose_theta = 0.;
float d_err = 0., b_err = 0., h_err = 0.; // Distance error (m), bearing error (rad), heading error (rad)
float phi_l = 0., phi_r = 0.; // Wheel rotation (radians)


// Wheel rotation vars
float left_speed_pct = 0.;
float right_speed_pct = 0.;
int left_dir = DIR_CCW;
int right_dir = DIR_CW;
int left_wheel_rotating = NONE;
int right_wheel_rotating = NONE;

// X and Theta Updates (global for debug output purposes)
// and their respective feedback controller gains
const float distance_gain = 1.;
const float theta_gain = 1.;
float dX  = 0., dTheta = 0.;

float to_radians(double deg) {
  return  deg * 3.1415/180.;
}

float to_degrees(double rad) {
  return  rad * 180 / 3.1415;
}

void setup() {
  sparki.servo(SERVO_CENTER);
  sparki.RGB(RGB_GREEN);
  delay(1000);

  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;

  // Set test cases here!
  set_pose_destination(.1,.1, to_radians(180));  // Goal_X_Meters, Goal_Y_Meters, Goal_Theta_Degrees
}

// Sets target robot pose to (x,y,t) in units of meters (x,y) and radians (t)
void set_pose_destination(float x, float y, float t) {
  dest_pose_x = x;
  dest_pose_y = y;
  dest_pose_theta = t;
  if (dest_pose_theta > 2. * M_PI) dest_pose_theta -= 2. * M_PI;
  if (dest_pose_theta < -2. * M_PI) dest_pose_theta += 2. * M_PI;
  orig_dist_to_goal = 0; // TODO
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
}


void updateOdometry() {
  // TODO: Update pose_x, pose_y, pose_theta
  pose_theta += ((right_speed_pct * ROBOT_SPEED * CYCLE_TIME) - (left_speed_pct * ROBOT_SPEED * CYCLE_TIME)) / AXLE_DIAMETER;
  pose_x += (((right_speed_pct * ROBOT_SPEED * CYCLE_TIME) + (left_speed_pct * ROBOT_SPEED * CYCLE_TIME)) / 2) * cos(pose_theta);
  pose_y += (((right_speed_pct * ROBOT_SPEED * CYCLE_TIME) + (left_speed_pct * ROBOT_SPEED * CYCLE_TIME)) / 2) * sin(pose_theta);

  // Bound theta
  if (pose_theta > 2 * M_PI) pose_theta -= 2.*M_PI;
  if (pose_theta <= -2 * M_PI) pose_theta += 2.*M_PI;
}

void displayOdometry() {
  sparki.print("X: ");
  sparki.print(pose_x);
  sparki.print(" Xg: ");
  sparki.println(dest_pose_x);
  sparki.print("Y: ");
  sparki.print(pose_y);
  sparki.print(" Yg: ");
  sparki.println(dest_pose_y); 
  sparki.print("T: ");
  sparki.print(to_degrees(pose_theta));
  sparki.print(" Tg: ");
  sparki.println(to_degrees(dest_pose_theta));

  sparki.print("dX : ");
  sparki.print(dX );
  sparki.print("   dT: ");
  sparki.println(dTheta);
  sparki.print("phl: "); sparki.print(phi_l); sparki.print(" phr: "); sparki.println(phi_r);
  sparki.print("p: "); sparki.print(d_err); sparki.print(" a: "); sparki.println(to_degrees(b_err));
  sparki.print("h: "); sparki.println(to_degrees(h_err));  
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
   
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      // Useful for testing odometry updates
      readSensors();
      if (line_center < threshold) {
        sparki.moveForward();
      } else if (line_left < threshold) {
        // TODO: Fill in odometry code
        sparki.moveLeft();
      } else if (line_right < threshold) {
        // TODO: Fill in odometry code
        sparki.moveRight();
      } else {
        sparki.moveStop();
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      } 
      break;
    case CONTROLLER_GOTO_POSITION_PART2:
      // TODO: Implement solution using moveLeft, moveForward, moveRight functions
      if (pose_x != dest_pose_x && pose_y != dest_pose_y && pose_theta != dest_pose_theta){
      b_err = atan2(dest_pose_y - pose_y, dest_pose_x - pose_x) - pose_theta;
      d_err = sqrt(pow((dest_pose_x - pose_x), 2.0) + pow((dest_pose_y - pose_y), 2.0));
      
      sparki.moveLeft(to_degrees(b_err));
      pose_theta += b_err;
      h_err = dest_pose_theta - pose_theta;
      delay(1000);
      //distance to move
      sparki.moveForward(d_err * 100);
      delay(1000);
      //change heading      
      sparki.moveLeft(to_degrees(dest_pose_theta - pose_theta));
      pose_x = dest_pose_x;
      pose_y = dest_pose_y;
      pose_theta = 0;
      dest_pose_theta = pose_theta;
      delay(1000);
      }
      break;      
    case CONTROLLER_GOTO_POSITION_PART3:      
      updateOdometry();
      b_err = atan2(dest_pose_y - pose_y, dest_pose_x - pose_x) - pose_theta;
      d_err = sqrt(pow((dest_pose_x - pose_x), 2.0) + pow((dest_pose_y - pose_y), 2.0));
      h_err = dest_pose_theta - pose_theta;
      if (d_err > 0.01 || h_err > .08) { //0.8 radians = 5 degrees
        dX = 0.1 * d_err;
        dTheta = 0.1 * b_err;
        if (d_err < 0.02){
          dTheta = 0.1 * h_err;
        }
        phi_l = (dX - ((AXLE_DIAMETER * dTheta)/2)) / WHEEL_RADIUS;
        phi_r = (dX + ((AXLE_DIAMETER * dTheta)/2)) / WHEEL_RADIUS;
        float maxOfRotation = max(abs(phi_l), phi_r);
        left_speed_pct = phi_l / (maxOfRotation);
        right_speed_pct = phi_r / (maxOfRotation);
        // TODO: Implement solution using motorRotate and proportional feedback controller.
        // sparki.motorRotate function calls for reference:
        if (phi_l < 0){
          sparki.motorRotate(MOTOR_LEFT, right_dir, int(abs(left_speed_pct) * 100));
        }
        else {
          sparki.motorRotate(MOTOR_LEFT, left_dir, int(abs(left_speed_pct) * 100));
        }
        sparki.motorRotate(MOTOR_RIGHT, right_dir, int(right_speed_pct * 100));
      }
      else {
        sparki.moveStop();
        sparki.RGB(RGB_RED);
        left_speed_pct = 0;
        right_speed_pct = 0;
      }
      break;
  }
  sparki.clearLCD();
  displayOdometry();
  sparki.updateLCD();

  

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(10);
}
