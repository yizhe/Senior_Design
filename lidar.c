#include "urg_sensor.h"
#include "urg_utils.h"
#include "urg_connection.h"
#include <stdio.h>
#include <stdlib.h>

#define NUM_DIR 18
#define NUM_DIST 8
//occupancy map for the environment, resolution is 10 deg * 1meter 
int occ_map[NUM_DIR][NUM_DIST];

void initiate_occ(){
  int i,j;
  for (i=0; i<NUM_DIR; ++i)
    for (j=0; j<NUM_DIR; ++j)    
      occ_map[i][j] = 0;
}

void update_occ (long* range_data){
  printf("updating! \n");
}

int main(void){
  int MEASURE_TIMES = 1,i;
  urg_t* urg = (urg_t*)malloc(sizeof(urg_t));
  int ret;
  long *length_data;
  int length_data_size;
  //connect and open device 
  //device is mounted on /dev/ttyACM0
  const char connect_device[] = "/dev/ttyACM0";
  const long connect_baudrate = 115200;
  if (urg_open(urg, URG_SERIAL,connect_device, connect_baudrate) < 0){
    return 1;
  }
  initiate_occ();

  length_data = (long*)malloc(sizeof(long) *urg_max_data_size(urg));
  // \todo check length_data is not NULL
  ret = urg_start_measurement(urg, URG_DISTANCE, 0, 0);
  // \todo check
  // error code
  for (i=0; i<MEASURE_TIMES; ++i){
    length_data_size = urg_get_distance(urg, length_data, NULL);
    for (i=0; i<length_data_size; i=i+5){
      printf("data %d: %ld\n", i, length_data[i]);
    }
  }
  update_occ(length_data);
  urg_close(urg);
  free(length_data);
  return 0;
}
