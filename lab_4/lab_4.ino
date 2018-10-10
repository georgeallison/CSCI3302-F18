#include <sparki.h>

#define ROBOT_SPEED 0.0278
#define MIN_CYCLE_TIME .100
#define AXLE_DIAMETER 0.0865
#define FWD 1
#define NONE 0
#define BCK -1

// Screen size
#define SCREEN_X_RES 128.
#define SCREEN_Y_RES 64.

// Map size
#define NUM_X_CELLS 4
#define NUM_Y_CELLS 4

// Start line is 18", 2" from bottom left corner
#define START_LINE_X .4572 
#define START_LINE_Y .0508 

#define SERVO_POS_DEG 45

int current_state = 1;
const int threshold = 800;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;
float distance = 0.;
unsigned long last_cycle_time = 0;


float pose_x = 0., pose_y = 0., pose_theta = 0., pose_servo = 0.;
int left_wheel_rotating = 0, right_wheel_rotating = 0;

// TODO: Define world_map multi-dimensional array
bool world_map[4][4]



// TODO: Figure out how many meters of space are in each grid cell
const float CELL_RESOLUTION_X = 15;  // Line following map is ~60cm x ~42cm
const float CELL_RESOLUTION_Y = 10; // Line following map is ~60cm x ~42cm


void setup() {
  pose_x = START_LINE_X;
  pose_y = START_LINE_Y;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  pose_servo = to_radians(SERVO_POS_DEG);

  sparki.servo(-to_degrees(pose_servo)); // Servo axis points down instead of up, so we need to negate it to be consistent with the robot's rotation axis

  // TODO: Initialize world_map

  sparki.clearLCD();
  displayMap();
  delay(1000);  
  last_cycle_time = millis();
}

float to_radians(float deg) {
  return deg * 3.14159 / 180.;
}

float to_degrees(float rad) {
  return rad * 180. / 3.14159;
}

// Ultrasonic Sensor Readings -> Robot coordinates
void transform_us_to_robot_coords(float dist, float theta, float *rx, float *ry) {
  *rx = dist*sin(theta);
  *ry = dist*cos(theta);
  
}

// Robot coordinates -> World frame coordinates
void transform_robot_to_world_coords(float x, float y, float *gx, float *gy) {
  *gx = rx * cos(pose_theta) - ry * sin(pose_theta) + pose_x;
  *gy = rx * sin(pose_theta) + ry * cos(pose_theta) + pose_y;
}

bool transform_xy_to_grid_coords(float x, float y, int *i, int *j) {
  if (x < 0 || y <0) {
    return 1;
  } else if (x > 40 || y > 60){
    return 2;
  }

  *j = floor(x/15);
  *j = floor(y/10);
  
  return 0;
}

// Turns grid coordinates into world coordinates (grid centers)
bool transform_grid_coords_to_xy(int i, int j, float *x, float *y) {
  // TODO: Return 0 if the I,J coordinates were out of bounds
  if (i < 0 || j < 0) {
    return 1;
  } else if (i > 15 || j > 10) {
    return 1;
  ]
    // TODO: Set *x and *y
   *x = floor(i*15) + 7;
   *y = floor(i*10) + 5; 

   return 0;
 


  
  return 1;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  distance = float(sparki.ping()) / 100.;
}

void moveRight() {
  left_wheel_rotating = FWD;
  right_wheel_rotating = BCK;
  sparki.moveRight();
}

void moveLeft() {
  left_wheel_rotating = BCK;
  right_wheel_rotating = FWD;
  sparki.moveLeft();
}

void moveForward() {
  left_wheel_rotating = FWD;
  right_wheel_rotating = FWD;
  sparki.moveForward();
}

void moveBackward() {
  left_wheel_rotating = BCK;
  right_wheel_rotating = BCK;
  sparki.moveBackward();
}

void moveStop() {
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  sparki.moveStop();
}

void updateOdometry(float cycle_time) {
  pose_x += cos(pose_theta) * cycle_time * ROBOT_SPEED * (left_wheel_rotating + right_wheel_rotating)/2.;
  pose_y += sin(pose_theta) * cycle_time * ROBOT_SPEED * (left_wheel_rotating + right_wheel_rotating)/2.;
  pose_theta += (right_wheel_rotating - left_wheel_rotating) * cycle_time * ROBOT_SPEED / AXLE_DIAMETER;
}

void displayMap() {
  // TODO: Measure how many pixels will be taken by each grid cell
  const int PIXELS_PER_X_CELL = 0;
  const int PIXELS_PER_Y_CELL = 0; 
  int cur_cell_x=-1, cur_cell_y=-1;

  // TODO: Make sure that if the robot is "off-grid", e.g., at a negative grid position or somewhere outside your grid's max x or y position that you don't try to plot the robot's position!
  
  // TODO: Draw Map
}

void serialPrintOdometry() {
  Serial.print("\n\n\nPose: ");
  Serial.print("\nX: ");
  Serial.print(pose_x);
  Serial.print("\nY: ");
  Serial.print(pose_y);
  Serial.print("\nT: ");
  Serial.print(pose_theta * 180. / 3.14159);
  Serial.print("\n");
}

void displayOdometry() {
  sparki.println("Pose: ");
  sparki.print("X: ");
  sparki.println(pose_x);
  sparki.print("Y: ");
  sparki.println(pose_y);
  sparki.print("T: ");
  sparki.println(pose_theta * 180. / 3.14159);
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long begin_movement_time = 0;
  unsigned long end_time = 0;
  unsigned long delay_time = 0;
  bool found_object = 0;
  readSensors();
  
  sparki.clearLCD();
  last_cycle_time = (millis() - last_cycle_time) / 1000;
  updateOdometry(last_cycle_time);
  last_cycle_time = millis(); // Start timer for last motor command to determine cycle time
  serialPrintOdometry();

  // Mapping Code
  sparki.servo(-to_degrees(pose_servo));

  // TODO: Check if sensors found an object
  if(distance > 0){
     float rx, ry, gx, gy, i, j;
     transform_us_to_robot_coords(distance, SERVO_POS_DEG, &rx, &ry);
     transform_robot_to_world_coords(rx, ry, &gx, &gy);
     transform_xy_to_grid_coords(gx, gy, &i, &j);
     world_map[i, j] = true;
  }
  // TODO: Adjust Map to accommodate new object

  displayMap();

  if (line_center < threshold) {
    moveForward();
  } else if (line_left < threshold) {
    moveLeft();
  } else if (line_right < threshold) {
    moveRight();
  } else {
    moveStop();
  }
  
  // Check for start line, use as loop closure
  // NOTE: Assumes robot is moving counter-clockwise around the map (by setting pose_theta = 0)!
  //       If your robot is moving clockwise, set pose_theta to pi radians (i.e., pointing left).
  if (line_left < threshold && line_right < threshold && line_center < threshold) {
    pose_x = START_LINE_X;
    pose_y = START_LINE_Y;
    pose_theta = 0.;
  } 

  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*MIN_CYCLE_TIME)
    delay(1000*MIN_CYCLE_TIME - delay_time); // make sure each loop takes at least MIN_CYCLE_TIME ms
  else
    delay(10);
}
