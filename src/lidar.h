#ifndef LIDAR_H
#define LIDAR_H


#include "Serial.h"
#include "string.h"
#include "stdlib.h"
#include <unistd.h>

#define INVALID_DISTANCE 99999

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif




// -----------------------

class LIDARLite : public Laser
{

public:

  Serial serial;
  int angle;
  int distance;
  bool inSync;
  int counter;

  // scan_size, scan_rate_hz, detection_angle_degrees, distance_no_detection_mm, detection_margin, offset_mm
  LIDARLite(std::string device): Laser(360, 5, 360, 40000, 0, 0  )
  {
    angle = -1;
    serial.begin((char*)device.c_str(), 115200);
    distance = 0;
    inSync = false;
    counter = 0;
  }

  ~LIDARLite(){
    serial.end();
  }

  void transferScan(int *scanvals){
    do {
      readScan(scanvals);
    } while (counter <= 1);
  }

  void readScan(int *scanvals){
    int sync = 0;
    int bytesRead;
    unsigned char data;
    // make all rays invalid
    for (int k=0; k<scan_size; k++) scanvals[k] = INVALID_DISTANCE;

    int j = 0;
    int angle = 0;
    int distance = 0;
    while (true){
      bytesRead = serial.available();
      if (bytesRead > 0 ){
        data = serial.sread();
        if ((data == 0xCC) && (sync == 0)) sync++;
          else if ((data == 0xDD) && (sync == 1)) sync++;
          else if ((data == 0xEE) && (sync == 2)) sync++;
          else if ((data == 0xFF) && (sync == 3)) sync++;
          else sync = 0;
        if (sync == 4) {
          inSync = true;
          counter++;
          //printf("-----sync----\n");
          break;
        }
        if (inSync){
          if (j == 0){
            angle = (data << 8);
            j++;
          } else if (j == 1){
            angle |= data;
            j++;
          } else if (j == 2){
            distance = (data << 8);
            j++;
          } else if (j == 3){
            distance |= data;
            j = 0;
            //printf("angle=%d, distance=%d\n", angle, distance);
            if (distance > 0) {
              if ((angle >=0 ) && (angle < scan_size)){
                //scanvals[angle]=2000 + rand() % 50; //distance*10 +35;
                //if (distance*10+35 < distance_no_detection_mm)
                scanvals[angle]=distance*10 +35;
              }
              /*for (int i=0; i < 12; i++){
                int a = ((int)((angle-6+i)/12))*12;
                if ((a >=0 ) && (a < scan_size)){
                  scanvals[a]=distance*10 +35;
                }
              }*/
            }
          }
        }
      }
    }
  }

};


// -----------------------

class LidarSim : public Laser
{

public:
  float x;
  float y;
  int mx;
  int my;
  unsigned char lmap[500][500];
  // scan_size, scan_rate_hz, detection_angle_degrees, distance_no_detection_mm, detection_margin, offset_mm
  LidarSim(void): Laser(360, 5, 360, 25000, 0, 0  )
  {
    printf("scan_size %d\n", scan_size);
    x = 250;
    y = 250;
    mx = 1;
    my = 1;
    memset(lmap, 0, sizeof(lmap));
    for (int i=0; i < 1000; i++){
      float a = ((float)i)/500.0 * M_PI;
      int px = x+cos(a)*100.0;
      int py = y+sin(a)*100.0;
      //printf("%d,%d\n", px, py);
      lmap[px][py] = 255;
      /*lmap[px+1][py] = 255;
      lmap[px][py+1] = 255;*/
    }
    //getchar();
  }

  ~LidarSim(){

  }

  void transferScan(int *scanvals){
    // make all rays invalid
    for (int k=0; k<scan_size; ++k) scanvals[k] = INVALID_DISTANCE;
    //printf("scan_size %d\n", scan_size);
    //int kstart = rand() % 12;
    int kstart = 0;
    for (int k=kstart; k<scan_size; k+=12) {
      float cx = x;
      float cy = y;
      float a = ((float)k)/180.0*M_PI;
      //lmap[250][250] = 255;
      //printf("map=%d\n", lmap[250][250]);
      float stepx = cos(a)*1;
      float stepy = sin(a)*1;
      int px;
      int py;
      int dist = INVALID_DISTANCE;
      while (true){
        px = cx;
        py = cy;
        //printf("%d,%d\n", px, py);
        //getchar();
        if (lmap[px][py] == 255) {
          dist = sqrt((cy-y)*(cy-y) + (cx-x)*(cx-x)) * 20 + rand() % 100;
          //printf("dist=%d\n", dist);
          //getchar();
          break;
        }
        if ((px < 0) || (px >= 500)) break;
        if ((py < 0) || (py >= 500)) break;
        cx += stepx;
        cy += stepy;
      }
      //getchar();
      if ((k >= 0) && (k < scan_size)) scanvals[k] = dist;
    }
    usleep(500000);
    //x += mx;
    //y += my;
    if (x > 300) mx = -mx;
    if (x < 200) mx = -mx;
    if (y > 300) my = -my;
    if (y < 200) my = -my;
  }
};

/*class TestLidar : public Laser
{

public:
	TestLidar(void): Laser(10, 1.0, 360, ){

	}

}*/

#endif
