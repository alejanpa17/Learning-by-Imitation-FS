/*
PROGRAM NAME - autonomous_vehicle.c

PROGRAMMER/S - Alejandro Parrado Arribas

DATE - Started  25/01/2020
     - Finished 25/01/2021

BUGS - Not found yet

DESCRIPTION - This version of the program controls the actuators of a formula student
              driverless racing car with a machine learning technique named learning
              by imitation.

CURRENT STATE - The car is able to complete a lap in the oval circuit.

POSSIBLE UPDATES -  -> ROS
                    -> KNN
LICENSE - MIT License
*/


#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/lidar.h>
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <webots/supervisor.h>
#include <webots/joystick.h>

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


// to be used as array indices
enum { X, Y, Z };

// Estructura de datos lista doblemente enlazada
// Doubly linked list struct
struct node
{
      WbCameraRecognitionObject data;
      double position_abs[3];
      struct node *prev;
      struct node *next;
};

// Lista doblemente enlazada para los conos azules
// Doubly linked list for blue cones
struct node *head_left = NULL;
struct node *last_left = NULL;
struct node *current_left = NULL;

// Lista doblemente enlazada para los conos amarillos
// Doubly linked list for yellow cones
struct node *head_right = NULL;
struct node *last_right = NULL;
struct node *current_right = NULL;



/*
Distancia 2D entre el coche y un cono
Distance 2D between the car and cone
Input:
  - position_vec = 3d coord of the car (array)
  - cone = 3d coord of the cone
Output:
  - distanceToCone = 2d distance between position_vec and cone
*/
double distanceToCone(const double *position_vec, struct node *cone){

      if(cone==NULL)
        return 0;

     return sqrt(pow(position_vec[X] - cone->position_abs[X], 2)+pow(position_vec[Z] - cone->position_abs[Z], 2));;
}



/*
Distancia 2D entre dos puntos
Distance 2D between two points
Input:
  - vec0 = 3d coord (array)
  - vec1 = 3d coord (array)
Output:
  - computeDistance = 2d distance between vec0 and vec1
*/
double computeDistance(const double *vec0, const double *vec1){

     return sqrt(pow(vec0[X] - vec1[X], 2)+pow(vec0[Z] - vec1[Z], 2));;
}



/*
Aniade un cono a la lista doblemente enlazada
Insert a cone in the doubly linked list
Input:
  - data = cone data
  - position_vec = coord 3d of the cone (array)
  - head = head node of doubly linked list
  - last = last node of doubly linked list
  - current = current node of doubly linked list
*/
void insert(WbCameraRecognitionObject data, const double *position_vec, struct node **head , struct node **last ,struct node **current)
{
      struct node *link = (struct node*) malloc(sizeof(struct node));
      link->data = data;
      link->position_abs[X] = position_vec[X];
      link->position_abs[Z] = position_vec[Z];
      link->prev = NULL;
      link->next = NULL;
      if(*head==NULL)
      {
            *head = link;
            return;
      }
      *current = *head;
      while((*current)->next!=NULL)
           *current = (*current)->next;
      (*current)->next = link;
      *last = link;
      link->prev = *current;
}



/*
Imprime la lista doblemente enlazada
Print the doubly linked list
Input:
  - head = head node of doubly linked list
*/
void printList(struct node *head)
{
      if(head==NULL)
        return;
      struct node *ptr = head;
      printf("\n[head] <=>");
      while(ptr->next != NULL)
     {
            //printf(" %d at %.2f, %.2f <=>",ptr->data.id, ptr->position_abs[X], ptr->position_abs[Z]);
            printf(" %d <=>",ptr->data.id);
            ptr = ptr->next;
     }
     //printf(" %d at %.2f, %.2f <=>",ptr->data.id, ptr->position_abs[X], ptr->position_abs[Z]);
     printf(" %d <=>",ptr->data.id);
     printf(" [last]\n");
}



/*
Comprueba si un cono esta en una lista doblemente enlazada
Check if a cone is in a doubly linked list
Input:
  - data = cone data
  - head = head node of doubly linked list
  - current = current node of doubly linked list
*/
int inList(WbCameraRecognitionObject data, struct node *head, struct node *current){

      if(head==NULL)
        return 0;
      current = head;
      while(current!=NULL){
        if(current->data.id == data.id){
          return 1;
        }
                 current = current->next;
      }
     return 0;
}



/*
Distancia del origen del coche a un segmento formado por dos conos, para una mayor explicación revisa en la tesis Algoritmo#2
Distance from the origin of the car to a segment formed by two cones, for further explanation check in the thesis Algorithm#2
Input:
  - p0 = 2d coord of cone 1
  - p1 = 2d coord of cone 2
  - p2 = 2d coord of projective
  - p3 = 2d coord of origin
Output:
  - get_distance_to_segment = distance from the origin of the car to a segment formed by two cones
*/
float get_distance_to_segment(float p0_x, float p0_y, float p1_x, float p1_y, float p2_x, float p2_y, float p3_x, float p3_y){


    //SEGMENT OF THE CONE
    float a1 = p1_y - p0_y;
    float b1 = p0_x - p1_x;
    float c1 = a1*(p0_x) + b1*(p0_y);

    //PROJECTION FROM THE CAR
    float a2 = p3_y - p2_y;
    float b2 = p2_x - p3_x;
    float c2 = a2*(p2_x)+ b2*(p2_y);

    float determinant = a1*b2 - a2*b1;

    if(determinant == 0)
        return 9998.0; //Colinear

    float i_x = (b2*c1 - b1*c2)/determinant;
    float i_y = (a1*c2 - a2*c1)/determinant;
    float distance = sqrt(pow(i_x - p3_x, 2)+pow(i_y - p3_y, 2)); //Distance origin to collision in segment
    //printf("Pto: %f %f\n", i_x, i_y);

    if( sqrt(pow(i_x - p2_x, 2)+pow(i_y - p2_y, 2)) > distance ) //Orientation desired
        return 9995.0;

    if( (p0_x < i_x && p1_x > i_x) || (p0_x > i_x && p1_x < i_x) )  //Point belongs to segment?
        return distance;

    return 9997.0; // No collision to segment

}



/*
Distancia desde el aleron delantero al limite del circuito, para una mayor explicación revisa en la tesis Algoritmo#1
Distance from front wing to circuit boundary, for further explanation check in the thesis Algorithm#1
DEPRECATED
Input:
  - head = head node of doubly linked list
  - last = last node of doubly linked list
  - current = current node of doubly linked list
  - reference = id the corner of the front wing 0 -> Left Front / 1 -> Rigth Front
Output:
  - lateralDistance = distance from the origin of the car to a segment formed by two cones
*/
double lateralDistance(struct node *head , struct node *last ,struct node *current, int reference){

      if(last==NULL)
        return -1;

      //MFT00 by default
      const double *position_vec = wb_supervisor_node_get_position(wb_supervisor_node_get_from_def("MFT00"));

      //Esquina izquierda
      if(reference == 0)
        position_vec = wb_supervisor_node_get_position(wb_supervisor_node_get_from_def("MFT00.FL"));

      //Esquina derecha
      if(reference == 1)
        position_vec = wb_supervisor_node_get_position(wb_supervisor_node_get_from_def("MFT00.FR"));

      //Buscar el cono mas cercano a la esquina
      current = head;
      struct node *closest = current;
      double closest_dist = 9999;

      while(current!=NULL)
      {
           double current_dist = sqrt(pow(position_vec[X] - current->position_abs[X], 2) + pow(position_vec[Z] - current->position_abs[Z], 2));
           if(closest_dist > current_dist){
             closest = current;
             closest_dist = current_dist;
           }
           current = current->next;
      }


      //Segundo cono mas cercano
      double closest_next = 9999;
      double closest_prev = 9999;

      if(closest->next != NULL && closest->prev != NULL){
        closest_next = sqrt(pow(position_vec[X] - closest->next->position_abs[X], 2) + pow(position_vec[Z] - closest->next->position_abs[Z], 2));
        closest_prev = sqrt(pow(position_vec[X] - closest->prev->position_abs[X], 2) + pow(position_vec[Z] - closest->prev->position_abs[Z], 2));
      }

      else{
        return -1;
      }

      if(closest_next < closest_prev)
      {
        //printf("Closest cones: %d <=> %d \n",closest->data.id, closest->next->data.id);
        double a = closest->next->position_abs[Z] - closest->position_abs[Z];
        double b = closest->position_abs[X] - closest->next->position_abs[X];
        double c = -(a * (closest->position_abs[X]) + b * (closest->position_abs[Z]));

        return fabs((a * position_vec[X] + b * position_vec[Z] + c)) / (sqrt(a * a + b * b));
      }

      if(closest_next > closest_prev)
      {
        //printf("Closest cones: %d <=> %d \n",closest->data.id, closest->prev->data.id);
        double a = closest->prev->position_abs[Z] - closest->position_abs[Z];
        double b = closest->position_abs[X] - closest->prev->position_abs[X];
        double c = -(a * (closest->position_abs[X]) + b * (closest->position_abs[Z]));

        return fabs((a * position_vec[X] + b * position_vec[Z] + c)) / (sqrt(a * a + b * b));
      }

     return closest_dist;
}




// Racing Wheel Configuration
#define NAXIS 3
#define NAXISBOUNDS 2
#define NBUTTONS 13
#define NGAINS 2
int mGear;

#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})

#define min(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b;       \
})

//[Axis]
static const int Steering = 0;
static const int Throttle = 2;
static const int Brake    = 1;

//[AxisBounds]
static const int minSteering = -32768;
static const int maxSteering = 32767;
static const int minThrottle = 32767;
static const int maxThrottle = -32768;
static const int minBrake = 32767;
static const int maxBrake = -32767;

//[Buttons]
static const int PreviousGear        =  5;
static const int NextGear       =  4;


//[Gains]
static const double AutoCentering = 15000.0;
static const double Resistance = 3.0;



double convertFeedback(int raw, int minimum, int maximum) {
  if (maximum == minimum){
    printf("Prevent division by 0.\n");
    return 0;
  }

  return max(0.0, min(1.0, ((double)(raw - minimum)) / ((double)(maximum - minimum))));
}

void init() {

  wbu_driver_set_gear(1);
  wbu_driver_set_wiper_mode(DOWN);
  wb_joystick_enable(wb_robot_get_basic_time_step());
  wbu_driver_step();

  if (wb_joystick_is_connected())
     printf("%s detected\n", wb_joystick_get_model());
}



// Controla el coche con un volante Logitech, debe conectarse a la PC
// Control the car with a Logitech steering wheel, must connect to PC
void racing_wheel_logitech() {

  // update steering, throttle, and brake based on axes value

  // raw data
  int steeringFeedback = wb_joystick_get_axis_value(Steering);
  int throttleFeedback = wb_joystick_get_axis_value(Throttle);
  int brakeFeedback = wb_joystick_get_axis_value(Brake);

  // bounded scaled data [0, 1]
  double steeringAngle = convertFeedback(steeringFeedback, minSteering, maxSteering);
  double throttle = convertFeedback(throttleFeedback, minThrottle, maxThrottle);
  double brake = convertFeedback(brakeFeedback, minBrake, maxBrake);
  // useful debug code: display the resulting scaled data before sending it to the driver library
  // cout << "steering:" << steeringAngle << " throttle:" << throttle << " brake:" << brake << endl;

  // to automobile API
  wbu_driver_set_steering_angle(steeringAngle - 0.5);  // convert to [-0.5, 0.5] radian range
  wbu_driver_set_throttle(throttle);
  wbu_driver_set_brake_intensity(brake);

  // update gear and indicator based on buttons state
  int button = wb_joystick_get_pressed_button();
  int gear = mGear;
  static bool wasSwitchingToNextGear = false;
  static bool wasSwitchingToPreviousGear = false;
  bool isSwitchingToNextGear = false;
  bool isSwitchingToPreviousGear = false;

  // Automatic Gear
  if( wbu_driver_get_rpm() > 4000 && mGear < 3)
    gear += 1;

  if( wbu_driver_get_rpm() < 2000 && mGear > 1)
    gear -= 1;

  // Manual Gear
  while (button >= 0) {
    if (button == NextGear) {
      if (!wasSwitchingToNextGear)
        gear += 1;
      isSwitchingToNextGear = true;
    }
    else if (button == PreviousGear) {
      if (!wasSwitchingToPreviousGear)
        gear -= 1;
      isSwitchingToPreviousGear = true;
    }

    button = wb_joystick_get_pressed_button();
  }

  wasSwitchingToNextGear = isSwitchingToNextGear;
  wasSwitchingToPreviousGear = isSwitchingToPreviousGear;

  static const double maxSpeed = 60.0;  // speed from which the max gain is applied
  gear = max(-1, min(wbu_driver_get_gear_number(), gear));

  // Gear Shift
  if (gear != mGear) {
    mGear = gear;
    printf("gear: %d\n", mGear);
    wbu_driver_set_gear(mGear);
  }

  double speed = wbu_driver_get_current_speed();
  if (AutoCentering > 0.0)
    wb_joystick_set_auto_centering_gain(speed > maxSpeed ? AutoCentering :
                                                       AutoCentering * speed / maxSpeed);
  if (Resistance > 0.0)
    wb_joystick_set_resistance_gain(speed > maxSpeed ? 0.0 : Resistance * (1.0 - speed / maxSpeed));

}




/*
Controla el coche sobre los actuadores dados los parametros de acelerador, freno y angulo del volante
Control the car over the actuators given the throttle, brake and steering angle parameters.
Input:
  - steeringAngle = [-0.5, 0.5] steering wheel angle
  - throttle = [0, 1] torque sent to the wheels
  - brake = [0, 1] brake intensity applied to the four wheels
*/
void autonomous_mlp(double steeringAngle, double throttle, double brake) {

  // to automobile API
  wbu_driver_set_steering_angle(steeringAngle);
  wbu_driver_set_throttle(throttle);
  wbu_driver_set_brake_intensity(brake);

  int gear = mGear;

  // Automatic Gear Shift
  if( wbu_driver_get_rpm() > 4000 && mGear < 3)
    gear += 1;
  if( wbu_driver_get_rpm() < 2000 && mGear > 1)
    gear -= 1;

  gear = max(-1, min(wbu_driver_get_gear_number(), gear));
  if (gear != mGear) {
    mGear = gear;
    printf("gear: %d\n", mGear);
    wbu_driver_set_gear(mGear);
  }

}


//Multilayer Perceptron
struct parameters
{
    int feature_size;
    int n_hidden;
    int hidden_layers_size;
    int output_layer_size;
    double*** weight;
};

void mat_mul(double* a, double** b, double* result, int n, int p) {
    // matrix a of size 1 x n (array)
    // matrix b of size n x p
    // matrix result of size 1 x p (array)
    // result = a * b
    //printf("%d %d\n", n, p);
    int j, k;
    for (j = 0; j < p; j++) {
        result[j] = + b[0][j]; //threshold
        //printf("%f\n", result[j] );
        for (k = 1; k < n; k++){
            //printf("%d %d\n", j, k);
            result[j] += (a[k-1] * b[k][j]);
        }
        //printf("%f\n", result[j] );
    }
}

void identity(int n, double* input, double* output) {
    //output[0] = 1; // Bias term

    for (int i = 0; i < n; i++){
        output[i] = input[i]; // Identity function
        //printf("%f\n", output[i]);
    }
}

void sigmoid(int n, double* input, double* output) {
    //output[0] = 1; // Bias term

    for (int i = 0; i < n; i++){
        output[i] = 1.0 / (1.0 + exp(- (input[i]))); // Sigmoid function
        //printf("%f\n", output[i]);
    }
}



/*
Propagación en adelante de un perceptrón multicapa
Forward propagation of a multilayer perceptron
Input:
  - param = multilayer perceptron
  - data = inputs for the multilayer perceptron (array)
  - min = min of the class to perform denormalization
  - max = max of the class to perform denormalization
Output:
  - forward_propagation = output of the multilayer perceptron
*/
double forward_propagation( struct parameters* param, double* data, double min, double max) {

    int n_layers = param->n_hidden + 2;
    int layer_sizes[3];
    layer_sizes[0] = param->feature_size - 1; //Input layer
    layer_sizes[1] = param->hidden_layers_size; //Hiddden layer
    layer_sizes[2] = param->output_layer_size; //Output layer

    // Create memory for arrays of inputs to the layers
    double** layer_inputs = (double**)calloc(n_layers, sizeof(double*));

    int i;
    for (i = 0; i < n_layers; i++)
        layer_inputs[i] = (double*)calloc(layer_sizes[i], sizeof(double));

    // Create memory for arrays of outputs from the layers
    double** layer_outputs = (double**)calloc(n_layers, sizeof(double*));

    for (i = 0; i < n_layers; i++)
        layer_outputs[i] = (double*)calloc(layer_sizes[i], sizeof(double));

    // Fill the input layer's input and output (both are equal) from data matrix with the given training example
    //layer_outputs[0][0] = 1; // Bias term of input layer
    for (i = 0; i < param->feature_size-1; i++){
        layer_outputs[0][i] = layer_inputs[0][i] = data[i];
     }

    //Perform forward propagation for each hidden layer
    //Calculate input and output of each hidden layer
    for (i = 1; i < n_layers-1; i++) {
        // //Compute layer_inputs[i]
        //printf("HOOLAA111\n");
        mat_mul(layer_outputs[i-1], param->weight[i-1], layer_inputs[i], layer_sizes[i-1]+1, layer_sizes[i]);
        // //Compute layer_outputs[i]
        sigmoid(layer_sizes[i], layer_inputs[i], layer_outputs[i]);


     }

    //Fill the output layers's input and output
    mat_mul(layer_outputs[n_layers-2], param->weight[n_layers-2], layer_inputs[n_layers-1], layer_sizes[n_layers-2]+1, layer_sizes[n_layers-1]);
    identity(layer_sizes[n_layers-1], layer_inputs[n_layers-1], layer_outputs[n_layers-1]);
           // //printf("HOO3333333333\n");

    //OUTPUT
    //printf("OUTPUT: %f    OUTPUT MLP: %f    DIFFERENCE: %f  \n", wbu_driver_get_steering_angle(), layer_outputs[n_layers-1][0], fabs(wbu_driver_get_steering_angle()) - fabs(layer_outputs[n_layers-1][0]) );
    double output =  (layer_outputs[n_layers-1][0] + 1)*(max - min) / 2 + min ;
    printf(" %f ", output);


    for (i = 0; i < n_layers; i++)
        free(layer_outputs[i]);

    free(layer_outputs);

    for (i = 0; i < n_layers; i++)
        free(layer_inputs[i]);

    free(layer_inputs);

    return output;

}



/*
Inicializacion perceptrón multicapa
Multilayer perceptron initialization
Input:
  - weight_txt = name of the file where the multilayer perceptron is stored
Output:
  - init_mlp = initialized multilayer perceptron struct
*/
struct parameters* init_mlp(char* weight_txt){

  struct parameters *param = (struct parameters*) malloc(sizeof(struct parameters));
  param->feature_size = 24;
  param->n_hidden = 1;
  param->hidden_layers_size = 7;
  param->output_layer_size = 1;
  int n_layers = param->n_hidden + 2;

  int i;

  // Save the sizes of layers in an array
  int* layer_sizes = (int*)calloc(n_layers, sizeof(int));

  layer_sizes[0] = param->feature_size - 1; //Input layer
  layer_sizes[1] = param->hidden_layers_size; //Hiddden layer
  layer_sizes[2] = param->output_layer_size; //Output layer


  FILE *in_file;
  //int number1, number2, sum;

  in_file = fopen(weight_txt, "r");

  if (in_file == NULL)  {
    printf("Error falta archivo con los pesos\n");
    return param;
  }

  // Create memory for the weight matrices between layers
  // weight is a pointer to the array of 2D arrays between the layers
  param->weight = (double***)calloc(n_layers - 1, sizeof(double**));

  // Each 2D array between two layers i and i+1 is of size ((layer_size[i]+1) x layer_size[i+1])
  // The weight matrix includes weights for the bias terms too
  for (i = 0; i < n_layers-1; i++)
      param->weight[i] = (double**)calloc(layer_sizes[i]+1, sizeof(double*));

  int j;
  for (i = 0; i < n_layers-1; i++){
      for (j = 0; j < layer_sizes[i]+1; j++){
          param->weight[i][j] = (double*)calloc(layer_sizes[i+1], sizeof(double));
       }
   }

   for (i = 0; i < n_layers-1; i++){
      for (j = 0; j < layer_sizes[i+1] ; j++){
        for (int k = 0; k < layer_sizes[i]+1; k++){
            fscanf(in_file, "%lf", &param->weight[i][k][j]);
        }
     }
   }

    // for (i = 0; i < n_layers-1; i++){
      // for (j = 0; j < layer_sizes[i]+1; j++){
        // for (int k = 0; k < layer_sizes[i+1]; k++){
          // printf("%f\t", param->weight[i][j][k]);
        // }
         // printf("\n");
     // }
   // }

  free(layer_sizes);
  fclose(in_file);
  return param;
}




#define TIME_STEP 50
#define UNKNOWN 99999.99


// enabe various 'features'
bool enable_collision_avoidance = false;
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;

// SICK laser
WbDeviceTag sick;
int sick_width = -1;
double sick_range = -1.0;
double sick_fov = -1.0;

// speedometer
WbDeviceTag display;
int display_width = 0;
int display_height = 0;
WbImageRef speedometer_image = NULL;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;
int manual_steering = 0;
bool autodrive = true;

// Odometer
void update_display() {
  const double NEEDLE_LENGTH = 50.0;

  // display background
  wb_display_image_paste(display, speedometer_image, 0, 0, false);

  // draw speedometer needle
  double current_speed = wbu_driver_get_current_speed();
  if (isnan(current_speed))
    current_speed = 0.0;
  double alpha = current_speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
  wb_display_draw_line(display, 100, 95, 100 + x, 95 + y);

  // draw text
  char txt[64];
  sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]);
  wb_display_draw_text(display, txt, 10, 130);
  sprintf(txt, "GPS speed:  %.1f", gps_speed);
  wb_display_draw_text(display, txt, 10, 140);
}

void compute_gps_speed() {
  const double *coords = wb_gps_get_values(gps);
  const double speed_ms = wb_gps_get_speed(gps);
  // store into global variables
  gps_speed = speed_ms * 3.6;  // convert from m/s to km/h
  memcpy(gps_coords, coords, sizeof(gps_coords));
}


int main(int argc, char **argv) {
  wbu_driver_init();

  init();

  const int num_pts = 13;
  const char *pts[num_pts];
  pts[0] = "MFT00.FL";
  pts[1] = "MFT00.B15";
  pts[2] = "MFT00.B30";
  pts[3] = "MFT00.B45";
  pts[4] = "MFT00.B60";
  pts[5] = "MFT00.B75";
  pts[6] = "MFT00.A90";
  pts[7] = "MFT00.A75";
  pts[8] = "MFT00.A60";
  pts[9] = "MFT00.A45";
  pts[10] = "MFT00.A30";
  pts[11] = "MFT00.A15";
  pts[12] = "MFT00.FR";


  // check if there is a SICK and a display
  int j = 0;
  for (j = 0; j < wb_robot_get_number_of_devices(); ++j) {
    WbDeviceTag device = wb_robot_get_device_by_index(j);
    const char *name = wb_device_get_name(device);
    if (strcmp(name, "Sick LMS 291") == 0)
      enable_collision_avoidance = true;
    else if (strcmp(name, "display") == 0)
      enable_display = true;
    else if (strcmp(name, "gps") == 0)
      has_gps = true;
    else if (strcmp(name, "camera") == 0)
      has_camera = true;
  }

  // camera device
  if (has_camera) {
    camera = wb_robot_get_device("camera");
    printf("Camara activada\n");
    wb_camera_enable(camera, TIME_STEP);
    wb_camera_recognition_enable(camera, TIME_STEP);
    camera_width = wb_camera_get_width(camera);
    camera_height = wb_camera_get_height(camera);
    camera_fov = wb_camera_get_fov(camera);
  }

  // SICK sensor
  if (enable_collision_avoidance) {
    sick = wb_robot_get_device("Sick LMS 291");
    printf("Lidar activado\n");
    wb_lidar_enable(sick, TIME_STEP);
    wb_lidar_enable_point_cloud(sick);
    sick_width = wb_lidar_get_horizontal_resolution(sick);
    sick_range = wb_lidar_get_max_range(sick);
    sick_fov = wb_lidar_get_fov(sick);
  }

  // initialize gps
  if (has_gps) {
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
  }

  // initialize display (speedometer)
  if (enable_display) {
    display = wb_robot_get_device("display");
    speedometer_image = wb_display_image_load(display, "speedometer.png");
  }



  int num_speed_reg = 10; // Number of the last speed registers
  int reg = 0;
  double speed_reg[num_speed_reg]; // Array of last speed registers

  // CSV data collected
  FILE *fpt;
  fpt = fopen("data.csv", "w+");
  fprintf(fpt,"FL, B15, B30, B45, B60, B75, A90, A75, A60, A45, A30, A15, FR, ");
  for(int i = 0; i < num_speed_reg; ++i){
         fprintf(fpt, "vel-%d, ",num_speed_reg - i);
  }
  fprintf(fpt, "steering_angle, throttle, brake\n");

  // Read necessary files
  struct parameters *param_steer = init_mlp("steer.txt");
  struct parameters *param_th = init_mlp("th.txt");
  struct parameters *param_br = init_mlp("br.txt");

  FILE *min_txt;
  min_txt = fopen("min.txt", "r");

  if (min_txt == NULL)  {
    printf("Error falta archivo para min\n");
  }

  double min[26];
  for (int i = 0; i < 26; i++){
    fscanf(min_txt, "%lf", &min[i]);
  }

  FILE *max_txt;
  max_txt = fopen("max.txt", "r");

  if (max_txt == NULL)  {
    printf("Error falta archivo para max\n");
  }

  double max[26];
  for (int i = 0; i < 26; i++){
    fscanf(max_txt, "%lf", &max[i]);
  }


  if(1){ // Checks if the multilayer perceptron is working correctly

            FILE *test_file;
            test_file = fopen("input_test_mlp.txt", "r");

            if (test_file == NULL)  {
              printf("Error falta archivo para el test\n");
            }

            int n_inputs = 23;
            double input_test_mlp[n_inputs];
            for (int i = 0; i < n_inputs; i++){
              fscanf(test_file, "%lf", &input_test_mlp[i]);
              input_test_mlp[i] = 2*(input_test_mlp[i] - min[i])/ (max[i] - min[i]) -1;  //NORMALIZAR
            }


            printf("WEBOTS: ");
            forward_propagation(param_steer, input_test_mlp, min[23], max[23]);

            double predicted_weka;
            fscanf(test_file, "%lf", &predicted_weka);
            printf("WEKA: %f ", predicted_weka);

            printf("\n\n\n");
            fclose(test_file);
   }


  // main loop
  while (wbu_driver_step() != -1) {

    // get user input
    if (wb_joystick_is_connected()){
      racing_wheel_logitech();
    }

    static int i = 0;

    const double *speed_vec = wb_supervisor_node_get_velocity(wb_supervisor_node_get_from_def("MFT00"));
    double speed_mod = sqrt(pow(speed_vec[0], 2) + pow(speed_vec[2], 2))*3.6;
    double downforce[3] = {0,-speed_mod*10,0};
    wb_supervisor_node_add_force(wb_supervisor_node_get_from_def("MFT00"), downforce, true);


    // updates sensors only every TIME_STEP milliseconds
    if (i % (int)(TIME_STEP / wb_robot_get_basic_time_step()) == 0 && has_camera) {

      //camera_image = wb_camera_get_image(camera);
      /* Get current number of object recognized */
      int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);

      const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);
      //printf("GPS: %.2f, %.2f NORMAL: %.2f, %.2f ", gps_coords[X], gps_coords[Z], position_vec[X], position_vec[Z]);

      int yellow = -1;
      double distance_yellow = 9999;
      int blue = -1;
      double distance_blue = 9999;
      const double *fl  = wb_supervisor_node_get_position(wb_supervisor_node_get_from_def("MFT00.FL"));
      const double *fr  = wb_supervisor_node_get_position(wb_supervisor_node_get_from_def("MFT00.FR"));

      // Detected cones
      for (i = 0; i < number_of_objects; ++i) {

        const double *position_vec = wb_supervisor_node_get_position(wb_supervisor_node_get_from_id(objects[i].id));

        // YELLOW
        if (objects[i].size[1] < 0.5 && objects[i].colors[1] == 1){ // if color = yellow

          double distance = computeDistance(position_vec, fr );

          if(!inList(objects[i], head_right, current_right) && distanceToCone(position_vec, last_right) < 4 && distance < distance_yellow){

            yellow = i; // closest yellow cone to the car not inserted to the list
            distance_yellow = distance;
          }
        }

        // BLUE
        if (objects[i].size[1] < 0.5 && objects[i].colors[2]  == 1){ // if color = blue

          double distance = computeDistance(position_vec, fl );

          if(!inList(objects[i], head_left, current_left) && distanceToCone(position_vec, last_left) < 4 && distance < distance_blue){

            blue = i; // closest blue cone to the car not inserted to the list
            distance_blue = distance;
          }
        }

        // ORANGE
        if (objects[i].size[1] < 0.5 && objects[i].colors[0]  == 1){ // if color = orange

          double distance_left = computeDistance(position_vec, fl );
          double distance_right = computeDistance(position_vec, fr );
          const double *position_vec = wb_supervisor_node_get_position(wb_supervisor_node_get_from_id(objects[i].id));

          // Insert the orange cone in the list of the corresponding side
          if (distance_left < distance_right){
            if(!inList(objects[i], head_left, current_left)){
              insert(objects[i], position_vec, &head_left, &last_left, &current_left);
            }
          }
          else{
            if(!inList(objects[i], head_right, current_right)){
              insert(objects[i], position_vec, &head_right, &last_right, &current_right);
            }
          }
        }
      } // end detected cones

      // Insert closest new cones to the list
      if (yellow != -1 && head_right!=NULL){
        const double *position_vec_yellow = wb_supervisor_node_get_position(wb_supervisor_node_get_from_id(objects[yellow].id));
        insert(objects[yellow], position_vec_yellow, &head_right, &last_right, &current_right);
      }
      if (blue != -1 && head_left!=NULL){
        const double *position_vec_blue = wb_supervisor_node_get_position(wb_supervisor_node_get_from_id(objects[blue].id));
        insert(objects[blue], position_vec_blue, &head_left, &last_left, &current_left);
      }


      // Get closest distances to the boundaries, for further explanation check in the thesis Algorithm#2
      int count = 0;
      float d[num_pts];

      for(i = 0; i < num_pts; ++i){

        const double *center  = wb_supervisor_node_get_position(wb_supervisor_node_get_from_def("MFT00.M"));
        const double *angle  = wb_supervisor_node_get_position(wb_supervisor_node_get_from_def(pts[i]));
        float min_distance = 9999.0;

        if(head_left!=NULL){

          struct node *ptr = head_left;

          while(ptr->next != NULL) {
            float distance = get_distance_to_segment( ptr->position_abs[X],  ptr->position_abs[Z],  ptr->next->position_abs[X],  ptr->next->position_abs[Z],  angle[X],  angle[Z],  center[X],  center[Z]);
            if(min_distance > distance){
              min_distance = distance;
            }
            ptr = ptr->next;
          }
        }

        if(head_right!=NULL){

          struct node *ptr = head_right;

          while(ptr->next != NULL) {
            float distance = get_distance_to_segment( ptr->position_abs[X],  ptr->position_abs[Z],  ptr->next->position_abs[X],  ptr->next->position_abs[Z],  angle[X],  angle[Z],  center[X],  center[Z]);
            if(min_distance > distance){
              min_distance = distance;
            }
            ptr = ptr->next;
          }
        }

        if(min_distance > 9990)
          count++;

        d[i] = min_distance;
        //printf("%s: %.2f  ", pts[i], d[i]);
        //fprintf(fpt,"%f, ", min_distance);
      }

      // Multilayer perceptron
      if(count == 0 && reg == num_speed_reg){

        // Prepare inputs for the multilayer perceptron
        double input_mlp[23];
        for(i = 0; i < num_pts; ++i){
          fprintf(fpt,"%f, ", d[i]);
          input_mlp[i] = 2*(d[i] - min[i])/ (max[i] - min[i]) -1;
        }

        for(int i = 0; i < num_speed_reg; ++i){
         fprintf(fpt, "%f, ",speed_reg[i]);
         input_mlp[i + num_pts] = 2*(speed_reg[i] - min[i + num_pts])/ (max[i + num_pts] - min[i + num_pts]) -1;
        }

        // for(int i = 0; i < 23; ++i){
        //   printf("%f, ",input_mlp[i]);
        // }printf("%f, %f, %f\n", wbu_driver_get_steering_angle(), wbu_driver_get_throttle(), wbu_driver_get_brake_intensity());

        // Inference the multilayer perceptron
        printf("STEERING: ");
        double steer = forward_propagation(param_steer, input_mlp, min[23], max[23]);
        printf("THROTTLE: ");
        double th = forward_propagation(param_th, input_mlp, min[24], max[24]);
        printf("BRAKE: ");
        double br = forward_propagation(param_br, input_mlp, min[25], max[25]);
        printf(" \n ");

        // Apply the outputs of the multilayer perceptron
        autonomous_mlp(steer, th, br);

        fprintf(fpt,"%f, %f, %f\n", wbu_driver_get_steering_angle(), wbu_driver_get_throttle(), wbu_driver_get_brake_intensity());
      }

      else{ // Default mode
        wbu_driver_set_throttle(0.1);
      }

      //fprintf(fpt,"\n");
      //fprintf(fpt,"\n");

      //printf(" \n ");
      // printList(head_left);
      // printList(head_right);
      // printf("\n\n");

      // update stuff
      if (has_gps)
        compute_gps_speed();
      if (enable_display)
        update_display();


      if(reg < num_speed_reg){
        speed_reg[reg] = wbu_driver_get_current_speed();
        reg += 1;
      }

      else{
        for(int i = 0; i < num_speed_reg - 1; ++i){
         speed_reg[i] = speed_reg[i+1];
        }
        speed_reg[num_speed_reg - 1] = wbu_driver_get_current_speed();
      }



    }

    ++i;
  } // End while

  wbu_driver_cleanup();

  return 0;  // ignored

} // End main
