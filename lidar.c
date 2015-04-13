#include "urg_sensor.h"
#include "urg_utils.h"
#include "urg_connection.h"
#include <stdio.h>
#include <stdlib.h>

#define RANGE 120
#define TOTALRANGE 270
#define NUM_DIR 24 
#define NUM_DIST 10
#define OCC_CAP 20
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

int main(void){
  urg_t* urg = (urg_t*)malloc(sizeof(urg_t));
  int i,ret;
  long *rawdata;
  int rawdata_size;
  //connect and open device 
  //device is mounted on /dev/ttyACM0
  const char connect_device[] = "/dev/ttyACM0";
  const long connect_baudrate = 115200;
  if (urg_open(urg, URG_SERIAL,connect_device, connect_baudrate) < 0){
    printf("Cannot Open Device!\n");
    return 1;
  }
  initiate_occ();

  rawdata = (long*)malloc(sizeof(long) *urg_max_data_size(urg));
  // \todo check rawdata is not NULL
  ret = urg_start_measurement(urg, URG_DISTANCE, 0, 0);
  // \todo check
  // error code
  while (1){
    rawdata_size = urg_get_distance(urg, rawdata, NULL);
    printf("Total data points: %d\n",rawdata_size);
    update_occ(rawdata,rawdata_size,1);
  }
  urg_close(urg);
  free(rawdata);
  return 0;
}
