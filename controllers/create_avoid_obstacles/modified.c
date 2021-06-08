/*--------------------include headers--------------------*/
//---standard headers---//
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
//---webots headers---//
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/receiver.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
/*--------------------device stuff--------------------*/
//---bumpers---//
#define BUMPERS_NUMBER 2
#define BUMPER_LEFT 0
#define BUMPER_RIGHT 1
static WbDeviceTag bumpers[BUMPERS_NUMBER];
static const char *bumpers_name[BUMPERS_NUMBER] = {"bumper_left", "bumper_right"};
//---sensors---//
#define CLIFF_SENSORS_NUMBER 4
#define CLIFF_SENSOR_LEFT 0
#define CLIFF_SENSOR_FRONT_LEFT 1
#define CLIFF_SENSOR_FRONT_RIGHT 2
#define CLIFF_SENSOR_RIGHT 3
static WbDeviceTag cliff_sensors[CLIFF_SENSORS_NUMBER];
static const char *cliff_sensors_name[CLIFF_SENSORS_NUMBER] = {"cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right"};
//---leds---//
#define LEDS_NUMBER 3
#define LED_ON 0
#define LED_PLAY 1
#define LED_STEP 2
static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_name[LEDS_NUMBER] = {"led_on", "led_play", "led_step"};
//---device tags---//
static WbDeviceTag receiver;
static const char *receiver_name = "receiver";
WbDeviceTag left_motor, right_motor, left_position_sensor, right_position_sensor;

/*--------------------misc stuff--------------------*/
//---speed---//
#define MAX_SPEED 16
#define NULL_SPEED 0
#define HALF_SPEED 8
#define MIN_SPEED -16
//---lengths---//
#define WHEEL_RADIUS 0.031
#define AXLE_LENGTH 0.271756
//#define AXLE_LENGTH 0.340000
#define ENCODER_RESOLUTION 507.9188

/*--------------------map stuff--------------------*/
#define OBSTACLE_MAX 1000
#define OBSTACLE_THRESHOLD 1.0 //size of an obstacle
#define BUFF_LEN 1024
#define MAX_POINTS 500
typedef struct obstacle {
	double xs[OBSTACLE_MAX], ys[OBSTACLE_MAX];
	int priority, count;
	double avgx, avgy;
} Obstacle;
static double dist = 0.0, orientation = 0.0, x = 0.0, y = 0.0; //map related variables
static Obstacle obstacles[OBSTACLE_MAX]; //list of obstacles
static int obstacle_count = 0;
static double angle_offset = 1.2275;
static int obstacle_reset = 0;

/*--------------------helper functions--------------------*/
static int get_time_step() {
	static int time_step = -1;
	if (time_step == -1)
		time_step = (int)wb_robot_get_basic_time_step();
	return time_step;
}

static void step() {
	if (wb_robot_step(get_time_step()) == -1) {
		wb_robot_cleanup();
		exit(EXIT_SUCCESS);
	}
}

static void init_devices() {
	int i;

	receiver = wb_robot_get_device(receiver_name);
	wb_receiver_enable(receiver, get_time_step());

	for (i = 0; i < LEDS_NUMBER; i++)
		leds[i] = wb_robot_get_device(leds_name[i]);

	for (i = 0; i < BUMPERS_NUMBER; i++) {
		bumpers[i] = wb_robot_get_device(bumpers_name[i]);
		wb_touch_sensor_enable(bumpers[i], get_time_step());
	}

	for (i = 0; i < CLIFF_SENSORS_NUMBER; i++) {
		cliff_sensors[i] = wb_robot_get_device(cliff_sensors_name[i]);
		wb_distance_sensor_enable(cliff_sensors[i], get_time_step());
	}

	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);
	wb_motor_set_velocity(left_motor, 0.0);
	wb_motor_set_velocity(right_motor, 0.0);

	left_position_sensor = wb_robot_get_device("left wheel sensor");
	right_position_sensor = wb_robot_get_device("right wheel sensor");
	wb_position_sensor_enable(left_position_sensor, get_time_step());
	wb_position_sensor_enable(right_position_sensor, get_time_step());
}

static bool is_there_a_collision_at_left() {
	return (wb_touch_sensor_get_value(bumpers[BUMPER_LEFT]) != 0.0);
}

static bool is_there_a_collision_at_right() {
	return (wb_touch_sensor_get_value(bumpers[BUMPER_RIGHT]) != 0.0);
}

static void fflush_ir_receiver() {
	while (wb_receiver_get_queue_length(receiver) > 0)
		wb_receiver_next_packet(receiver);
}

static bool is_there_a_cliff_at_left() {
	return (wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_LEFT]) < 100.0 || wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_LEFT]) < 100.0);
}

static bool is_there_a_cliff_at_right() {
	return (wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_RIGHT]) < 100.0 || wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_RIGHT]) < 100.0);
}

static bool is_there_a_cliff_at_front() {
	return (wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_LEFT]) < 100.0 || wb_distance_sensor_get_value(cliff_sensors[CLIFF_SENSOR_FRONT_RIGHT]) < 100.0);
}

static void go_forward() {
	wb_motor_set_velocity(left_motor, MAX_SPEED);
	wb_motor_set_velocity(right_motor, MAX_SPEED);
}

static void stop() {
	wb_motor_set_velocity(left_motor, -NULL_SPEED);
	wb_motor_set_velocity(right_motor, -NULL_SPEED);
}

static void passive_wait(double sec) {
	double start_time = wb_robot_get_time();
	do {
		step();
	} while (start_time + sec > wb_robot_get_time());
}

/*--------------------map functions--------------------*/
//---an original function that had to be modified---//
static void turn(double angle) {
	stop();
	obstacle_reset++;
	if (obstacle_reset == 5) {
		//angle = angle * angle_offset * 1.09; //1.01739
		orientation /= 1.04;
	 	obstacle_reset = 0;
	} 
	else
		angle = angle * angle_offset;// * 1.01739;	//the important part
	double l_offset = wb_position_sensor_get_value(left_position_sensor);
	double r_offset = wb_position_sensor_get_value(right_position_sensor);
	step();
	double neg = (angle < 0.0) ? -1.0 : 1.0;
	wb_motor_set_velocity(left_motor, neg * HALF_SPEED);
	wb_motor_set_velocity(right_motor, -neg * HALF_SPEED);
	double orient;
	do {
		double l = wb_position_sensor_get_value(left_position_sensor) - l_offset;
		double r = wb_position_sensor_get_value(right_position_sensor) - r_offset;
		double dl = l * WHEEL_RADIUS;				 // distance covered by left wheel in meter
		double dr = r * WHEEL_RADIUS;				 // distance covered by right wheel in meter
		orient = neg * (dl - dr) / AXLE_LENGTH;	// delta orientation in radian
		step();
	} while (orient < neg * angle);
	stop();
	step();
}

//---distance helper functions---//
// gets distance from the position sensors
static double get_distance_right() {
	double p = wb_position_sensor_get_value(right_position_sensor);
	double dist = p * WHEEL_RADIUS;
	return dist;
}
static double get_distance_left() {
	double p = wb_position_sensor_get_value(left_position_sensor);
	double dist = p * WHEEL_RADIUS;
	return dist;
}

//---obstacle functions---//
// recalculates average value of coordinates and priority of an obstacle
static void recalculate_obstacle(Obstacle* o) {
	int i;
	o->avgx = o->avgy = 0.0;
	for (i = 0; i < o->count; i++) {
		o->avgx += o->xs[i];
		o->avgy += o->ys[i];
	}
	o->avgx = o->avgx / (double) o->count;
	o->avgy = o->avgy / (double) o->count;
	if (o->count < 10)
		o->priority = 0;
	else
		o->priority = 1;
}
// is the current position within range of the input obstacle?
static bool within_range(Obstacle* o) {
	int dist = sqrt((abs(o->avgy - y) ^ 2) + (abs(o->avgx - x) ^ 2));
	return (dist <= OBSTACLE_THRESHOLD) ? true : false;
}

// is the current position within range of any obstacles?
static bool near_obstacle() {
	int i;
	bool near = false;
	for (i = 0; i < obstacle_count; i++) {
		if (sqrt((abs(obstacles[i].avgy - y) ^ 2) + (abs(obstacles[i].avgx - x) ^ 2)) <= 1) {
			near = true;
			break;
		}
	}
	return near;
}

// if current position is not near an obstacle, creates a new obstacle. else adds the coordinates to the nearest existing obstacle
static void record_obstacle() {
	int i;
	bool recorded = false;
	for (i = 0; i < obstacle_count; i++) {
		if (within_range(&obstacles[i])) {
			if (obstacles[i].count < OBSTACLE_MAX) {
				double x1 = x, y1 = y;
				obstacles[i].xs[obstacles[i].count] = x1;
				obstacles[i].ys[obstacles[i].count++] = y1;
				recalculate_obstacle(&obstacles[i]);
				recorded = true;
				break;
			}
			else
				break;
		}
	}
	if (!recorded) {
		Obstacle new;
		double x1 = x, y1 = y;
		new.xs[0] = x1; new.ys[0] = y1;
		new.count = 1;
		recalculate_obstacle(&new);
		obstacles[obstacle_count++] = new;
	}
}
/*static void print_obstacles() {
	if (obstacle_count > 0) {
		int i;
		printf("------------\n");
		for (i = 0; i < obstacle_count; i++) {
			printf("%.2lf,%.2lf has count %d and priority %d\n", obstacles[i].avgx, obstacles[i].avgy, obstacles[i].count, obstacles[i].priority);
		}
	}
	else
		return;
}*/

//---update values---//
// simply updates the orientation with respect to the x axis
static void update_orientation(double angle) {
	orientation += angle;
	if (orientation >= 2*M_PI)
		orientation = orientation - 2*M_PI;
	else if (orientation < 0)
		orientation = 2*M_PI + orientation;
	else 
		return;
}
// averages the distance covered by the 2 wheels, and uses the formula to update the x and y values (x = a cos theta and y = a sin theta)
static void update_map(double* curr_right_dist, double* curr_left_dist) {
	double dist_step = (fabs(*curr_right_dist - get_distance_right()) + fabs(*curr_left_dist - get_distance_left())) / 2;
	dist += dist_step;
	x += dist_step * cos(orientation);
	y += dist_step * sin(orientation);
	*curr_right_dist = get_distance_right();
	*curr_left_dist = get_distance_left();
}

/*
//---misc---//
static void print_map_info() {
	printf("(%.2lf, %.2lf), %.2lf at %.2lf\n", x, y, dist, orientation);
}*/

/*--------------------main--------------------*/
int main(int argc, char **argv) {
	wb_robot_init();
	printf("iRobot Create controller started...\n");
	init_devices();
	srand(time(NULL));
	wb_led_set(leds[LED_ON], true);
	passive_wait(0.5);
	
	double curr_left_dist = 0.0;
	double curr_right_dist = 0.0;

	while (true) {
		double angle = 0.9 * M_PI;
		bool near = false;
		int steps_without_forward = 0;
		if (is_there_a_collision_at_left() || is_there_a_cliff_at_left() || is_there_a_collision_at_right() || is_there_a_cliff_at_right() || is_there_a_cliff_at_front()) {
			record_obstacle();
			turn(angle);
			update_orientation(angle);
			update_map(&curr_right_dist, &curr_left_dist);
			//print_obstacles();
		}
		else if ((near = near_obstacle()) && steps_without_forward < 2) {
      			turn(angle);
			update_orientation(angle);
			update_map(&curr_right_dist, &curr_left_dist);
			steps_without_forward++;
		}
		else {
			go_forward();
			steps_without_forward--;
			update_map(&curr_right_dist, &curr_left_dist);
		}
		fflush_ir_receiver();
		//print_map_info();
		step();
	};

	return EXIT_SUCCESS;
}
