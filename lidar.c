#include "urg_sensor.h"
#include "urg_utils.h"
#include "urg_connection.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#define RANGE 120
#define TOTALRANGE 270
#define NUM_DIR 60 
#define NUM_DIST 9
#define OCC_CAP 5
#define ARMRANGE 300
#define ARMRESOLUTION 1024

//occupancy map for the environment, resolution is 10 deg * 1meter 
int occ_map[NUM_DIR][NUM_DIST];

void initiate_occ(){
  int i,j;
  for (i=0; i<NUM_DIR; ++i)
    for (j=0; j<NUM_DIR; ++j)    
      occ_map[i][j] = 0;
}

void update_occ (long* data, int data_size, int debug){
  long* avg_data = (long*)malloc(sizeof(long)*NUM_DIR); 
  int i,j;
  long partial_sum;
  int descard = (TOTALRANGE-RANGE)/2;
  int start_ind = data_size/TOTALRANGE*descard; 
  int end_ind = data_size - start_ind;
  int range = (end_ind - start_ind)/NUM_DIR;
  for (i=start_ind; i<=end_ind; ++i){
    if (data[i] < 200)
      data[i] = NUM_DIST*500;
  } 
  for (i=0; i<NUM_DIR; ++i){ 
    partial_sum = 0;
    for (j=0; j<range; ++j)
      partial_sum += data[start_ind+i*range+j];
    avg_data[i] = partial_sum/range/500;
  }
  for (i=0; i<NUM_DIR; ++i){
    for (j=0; j<NUM_DIST; ++j){
      if (j==avg_data[i]){
        occ_map[i][j] += 1;
        if (occ_map[i][j] > OCC_CAP)
          occ_map[i][j] = OCC_CAP;
      }
      else{
        occ_map[i][j] -= 1;
        if (occ_map[i][j] < 0)
          occ_map[i][j] = 0;
      }
    }
  }  
  if (debug){ 
    system("clear");
    printf("start: %d, end: %d, range: %d\n",start_ind,end_ind,range);
    print_occ();
  }
  free(avg_data);
}

int output_occ(int limit){
  int* close_data = (int*)malloc(sizeof(int)*NUM_DIR);
  int i,j,start=-1,end=-1,min=NUM_DIST; 
  for (i=0; i < NUM_DIR; ++i){
    close_data[i] = 0;
    for (j=0; j < NUM_DIST; ++j){
      if (occ_map[i][j] == OCC_CAP)
        break;
      else
        close_data[i] += 1;
    } 
  }
  for (i=0; i<NUM_DIR; ++i){
    if (close_data[i] < min) 
      min = close_data[i];
  }
  if (min <= limit){
    for (i=0; i<NUM_DIR; ++i){
      if (close_data[i] == min){
        if (start < 0){
          start = i;
          end = i;
        }else
          end = i;
      }else{
        if (start > 0)
          break;
      }
    }
  }
  free(close_data);
  return (start+end)/2;
}

void print_occ(){
  int i,j;
  for (i=0; i<NUM_DIR; ++i){
    printf("Angle %d: ",RANGE/NUM_DIR*i-RANGE/2);
    for (j=0; j<NUM_DIST; ++j){
      printf("%d  ",occ_map[i][j]);
    }
    printf("\n");
  }
}

int init_serial(){
  const char serial_device[] = "/dev/ttyACM1";
  struct termios options;
  int fd = open(serial_device, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1){
    return fd;
  }
  tcgetattr(fd, &options);
  cfsetispeed(&options, B9600);
  cfsetospeed(&options, B9600);
  options.c_cflag |= (CLOCAL | CREAD);
  tcsetattr(fd, TCSANOW, &options);
  if (write(fd, "Ss 1 520Xm 200X", 9) < 0)
    printf("write failed!");  


  return fd;
}

void move_arm(int fd, int degree){
  char* str = (char*)malloc(sizeof(char)*10);
  if (degree < -ARMRANGE/2 || degree > ARMRANGE/2)
    printf("%d is out of range!\n", degree);
  int target = (degree + ARMRANGE/2)*(ARMRESOLUTION-1)/ARMRANGE;
  int length = sprintf(str, "s 1 %dX", target);
  printf("%s - %d\n",str,length);
  if (write(fd, str, length) < 0)
    printf("write failed!"); 
  usleep(200000); 
}

int main(void){
  urg_t* urg = (urg_t*)malloc(sizeof(urg_t));
  int i,ret, fd, output, direction;
  FILE *file;
  long *rawdata;
  int rawdata_size;
  //connect and open device 
  //device is mounted on /dev/ttyACM0
  const char lidar_device[] = "/dev/ttyACM0";
  const long connect_baudrate = 115200;
   
  if (urg_open(urg, URG_SERIAL,lidar_device, connect_baudrate) < 0){
    printf("Cannot Open Lidar Device!\n");
    return 1;
  }
  
  fd = init_serial();
  if (fd == -1){
    printf("Cannot Open Serial!\n");
    return 1;
  }
    
  initiate_occ();
  rawdata = (long*)malloc(sizeof(long) *urg_max_data_size(urg));
  ret = urg_start_measurement(urg, URG_DISTANCE, 0, 0);
  while (1){
    rawdata_size = urg_get_distance(urg, rawdata, NULL);
    update_occ(rawdata,rawdata_size,1);
    output = output_occ(2);
      if (output > 0){
        direction =RANGE/NUM_DIR*output - RANGE/2;
        printf("Final Direction: %d\n", direction);  
        move_arm(fd, direction);
      }
      else{
        move_arm(fd, 0);
      }
  }
  urg_close(urg);
  free(rawdata);
  return 0;
}
