#include "urg_sensor.h"
#include "urg_utils.h"
#include "urg_connection.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#define RANGE 120
#define TOTALRANGE 270
#define NUM_DIR 30 
#define NUM_DIST 9
#define OCC_CAP 5
#define ARMRANGE 300
#define ARMRESOLUTION 1024
#define DISTUNIT 500

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
      data[i] = NUM_DIST*DISTUNIT;
  } 
  for (i=0; i<NUM_DIR; ++i){ 
    partial_sum = 0;
    for (j=0; j<range; ++j)
      partial_sum += data[start_ind+i*range+j];
    avg_data[i] = partial_sum/range/DISTUNIT;
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

int output_occ(int limit, int* output_dir){
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
  for (i=0; i<NUM_DIR; ++i)
    printf("%d ",close_data[i]);
  printf("\n");
  for (i=0; i<NUM_DIR; ++i){
    if (close_data[i] < min) 
      min = close_data[i];
  }
  printf("min vs limit: %d %d\n", min, limit);
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
  *output_dir = (start+end)/2;
  if (min<=limit && end-start>0)
    return min;
  else
    return -1;
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

int init_serial(char* serial_device){
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
  if (write(fd, "Ss 1 520Xm 100X qX", 17) < 0)
    printf("write failed!");  


  return fd;
}

void move_arm(int fd, int degree, int enable){
  char* str = (char*)malloc(sizeof(char)*15);
  int length;
  if (degree < -ARMRANGE/2 || degree > ARMRANGE/2)
    printf("%d is out of range!\n", degree);
  int target = (degree + ARMRANGE/2)*(ARMRESOLUTION-1)/ARMRANGE;
  if (enable)  
    length = sprintf(str, "s 1 %dXQX", target);
  else
    length = sprintf(str, "s 1 %dXqX", target);
  printf("%s - %d\n",str,length);
  if (write(fd, str, length) < 0)
    printf("write failed!"); 
  usleep(80000);  
}

int main(int argc, char* argv[]){
  urg_t* urg = (urg_t*)malloc(sizeof(urg_t));
  int lidar_num=0, serial_num=1, detect_bound=2, warning_bound=1,debug_on=0;
  int i, fd, dist, dir, direction, rawdata_size;
  long *rawdata;
  const long connect_baudrate = 115200;
  char *lidar_device = (char*)malloc(sizeof(char)*15);
  char *serial_device = (char*)malloc(sizeof(char)*15);
  
  if (argc > 1){
    lidar_num = argv[1][0] - '0';
    if (argc > 2){
      serial_num = argv[2][0] - '0';
      if (argc > 3){
        detect_bound = argv[3][0] - '0';
        if (argc > 4){
          warning_bound = argv[4][0] - '0';
          if (argc > 5){
            debug_on = argv[5][0] - '0';
          }
        }
      }
    } 
  }
  printf("detect bound: %d, warning_bound: %d\n", detect_bound, warning_bound);
  sprintf(lidar_device, "/dev/ttyACM%d", lidar_num);
  sprintf(serial_device, "/dev/ttyACM%d", serial_num);
  
  //connect and open lidar device 
  if (urg_open(urg, URG_SERIAL,lidar_device, connect_baudrate) < 0){
    printf("Cannot Open Lidar Device!\n");
    return 1;
  }
  //connect serial port 
  fd = init_serial(serial_device);
  if (fd == -1){
    printf("Cannot Open Serial!\n");
    return 1;
  }
  //initialize measurement 
  initiate_occ();
  rawdata = (long*)malloc(sizeof(long) *urg_max_data_size(urg));
  if(urg_start_measurement(urg, URG_DISTANCE, 0, 0) != 0){
    printf("Cannot get measurements!\n");
    return 1;
  }

  while (1){
    rawdata_size = urg_get_distance(urg, rawdata, NULL);
    update_occ(rawdata, rawdata_size, debug_on);
    dist = output_occ(detect_bound, &dir);
      if (dist >= 0){
        direction =RANGE/NUM_DIR*dir - RANGE/2;
        printf("Final Direction: %d\n", direction); 
        if (dist <= warning_bound)
          move_arm(fd, direction, 1);
        else 
          move_arm(fd, direction, 0);
      }
      else{
        move_arm(fd, 0, 0);
      }
  }
  urg_close(urg);
  free(lidar_device);
  free(serial_device);
  free(rawdata);
  return 0;
}
