#ifndef STRUCTDEF_H
#define STRUCTDEF_H

#define legn 4 //number of legs
#define coor 3 //x,y,z
#define itr 8 //number of iterations
#define timer 50

//Structure Definition

//Crawl
//struct gait_struct{
//  double March[itr][legn][coor]={{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 50.0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 50.0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 50.0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 50.0}}} ;
//  double FB[itr][legn][coor]={{{0, -60, 0}, {0, -20, 0}, {0, 20, 0}, {0, 60, 0}}, {{0, 0, 0}, {0, -40, 0}, {0, 0, 0}, {0, 40, 0}}, {{0, 60, 0}, {0, -60, 0}, {0, -20, 0}, {0, 20, 0}}, {{0, 40, 0}, {0, 0, 0}, {0, -40, 0}, {0, 0, 0}}, {{0, 20, 0}, {0, 60, 0}, {0, -60, 0}, {0, -20, 0}}, {{0, 0, 0}, {0, 40, 0}, {0, 0, 0}, {0, -40, 0}}, {{0, -20, 0}, {0, 20, 0}, {0, 60, 0}, {0, -60, 0}}, {{0, -40, 0}, {0, 0, 0}, {0, 40, 0}, {0, 0, 0}}};
//  double LR[itr][legn][coor]={{{-60, 0, 0}, {-20, 0, 0}, {20, 0, 0}, {60, 0, 0}}, {{0, 0, 0}, {-40, 0, 0}, {0, 0, 0}, {40, 0, 0}}, {{60, 0, 0}, {-60, 0, 0}, {-20, 0, 0}, {20, 0, 0}}, {{40, 0, 0}, {0, 0, 0}, {-40, 0, 0}, {0, 0, 0}}, {{20, 0, 0}, {60, 0, 0}, {-60, 0, 0}, {-20, 0, 0}}, {{0, 0, 0}, {40, 0, 0}, {0, 0, 0}, {-40, 0, 0}}, {{-20, 0, 0}, {20, 0, 0}, {60, 0, 0}, {-60, 0, 0}}, {{-40, 0, 0}, {0, 0, 0}, {40, 0, 0}, {0, 0, 0}}} ;
//  double RT[itr][legn][coor]={{{-30.51, 18.68, 0}, {9.8, -6.81, 0}, {9.39, 7.37, 0}, {-26.8, -23.69, 0}}, {{0.0, 0.0, 0}, {19.98, -13.05, 0}, {0.0, 0.0, 0}, {-18.33, -15.28, 0}}, {{26.8, -23.69, 0}, {30.51, -18.68, 0}, {-9.8, -6.81, 0}, {-9.39, -7.37, 0}}, {{18.33, -15.28, 0}, {0.0, 0.0, 0}, {-19.98, -13.05, 0}, {0.0, 0.0, 0}}, {{9.39, -7.37, 0}, {-26.8, 23.69, 0}, {-30.51, -18.68, 0}, {9.8, 6.81, 0}}, {{0.0, 0.0, 0}, {-18.33, 15.28, 0}, {0.0, 0.0, 0}, {19.98, 13.05, 0}}, {{-9.8, 6.81, 0}, {-9.39, 7.37, 0}, {26.8, 23.69, 0}, {30.51, 18.68, 0}}, {{-19.98, 13.05, 0}, {0.0, 0.0, 0}, {18.33, 15.28, 0}, {0.0, 0.0, 0}}};
//};

////Trot
struct gait_struct{
  double March[itr][legn][coor]={{{0, 0, 0.0}, {0, 0, 0.0}, {0, 0, 0.0}, {0, 0, 0.0}}, {{0, 0, 50.0}, {0, 0, 50.0}, {0, 0, 0.0}, {0, 0, 0.0}}, {{0, 0, 50.0}, {0, 0, 50.0}, {0, 0, 0.0}, {0, 0, 0.0}}, {{0, 0, 50.0}, {0, 0, 50.0}, {0, 0, 0.0}, {0, 0, 0.0}}, {{0, 0, 0.0}, {0, 0, 0.0}, {0, 0, 0.0}, {0, 0, 0.0}}, {{0, 0, 0.0}, {0, 0, 0.0}, {0, 0, 50.0}, {0, 0, 50.0}}, {{0, 0, 0.0}, {0, 0, 0.0}, {0, 0, 50.0}, {0, 0, 50.0}}, {{0, 0, 0.0}, {0, 0, 0.0}, {0, 0, 50.0}, {0, 0, 50.0}}};
  double FB[itr][legn][coor]={{{0, -60.0, 0}, {0, -60.0, 0}, {0, 60.0, 0}, {0, 60.0, 0}}, {{0, -30.0, 0}, {0, -30.0, 0}, {0, 30.0, 0}, {0, 30.0, 0}}, {{0, 0.0, 0}, {0, 0.0, 0}, {0, 0.0, 0}, {0, 0.0, 0}}, {{0, 30.0, 0}, {0, 30.0, 0}, {0, -30.0, 0}, {0, -30.0, 0}}, {{0, 60.0, 0}, {0, 60.0, 0}, {0, -60.0, 0}, {0, -60.0, 0}}, {{0, 30.0, 0}, {0, 30.0, 0}, {0, -30.0, 0}, {0, -30.0, 0}}, {{0, 0.0, 0}, {0, 0.0, 0}, {0, 0.0, 0}, {0, 0.0, 0}}, {{0, -30.0, 0}, {0, -30.0, 0}, {0, 30.0, 0}, {0, 30.0, 0}}};
  double LR[itr][legn][coor]={{{-60.0, 0, 0}, {-60.0, 0, 0}, {60.0, 0, 0}, {60.0, 0, 0}}, {{-30.0, 0, 0}, {-30.0, 0, 0}, {30.0, 0, 0}, {30.0, 0, 0}}, {{0.0, 0, 0}, {0.0, 0, 0}, {0.0, 0, 0}, {0.0, 0, 0}}, {{30.0, 0, 0}, {30.0, 0, 0}, {-30.0, 0, 0}, {-30.0, 0, 0}}, {{60.0, 0, 0}, {60.0, 0, 0}, {-60.0, 0, 0}, {-60.0, 0, 0}}, {{30.0, 0, 0}, {30.0, 0, 0}, {-30.0, 0, 0}, {-30.0, 0, 0}}, {{0.0, 0, 0}, {0.0, 0, 0}, {0.0, 0, 0}, {0.0, 0, 0}}, {{-30.0, 0, 0}, {-30.0, 0, 0}, {30.0, 0, 0}, {30.0, 0, 0}}};
  double RT[itr][legn][coor]={{{-30.51, 18.68, 0}, {30.51, -18.68, 0}, {26.8, 23.69, 0}, {-26.8, -23.69, 0}}, {{-14.84, 10.01, 0}, {14.84, -10.01, 0}, {13.92, 11.26, 0}, {-13.92, -11.26, 0}}, {{0.0, 0.0, 0}, {0.0, 0.0, 0}, {0.0, 0.0, 0}, {0.0, 0.0, 0}}, {{13.92, -11.26, 0}, {-13.92, 11.26, 0}, {-14.84, -10.01, 0}, {14.84, 10.01, 0}}, {{26.8, -23.69, 0}, {-26.8, 23.69, 0}, {-30.51, -18.68, 0}, {30.51, 18.68, 0}}, {{13.92, -11.26, 0}, {-13.92, 11.26, 0}, {-14.84, -10.01, 0}, {14.84, 10.01, 0}}, {{0.0, 0.0, 0}, {0.0, 0.0, 0}, {0.0, 0.0, 0}, {0.0, 0.0, 0}}, {{-14.84, 10.01, 0}, {14.84, -10.01, 0}, {13.92, 11.26, 0}, {-13.92, -11.26, 0}}};
};

/*
 //Johnson's Idea for Gait
 struct gait_struct{
  double March[itr][legn][coor]={{{0, 0, 50.0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 50.0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 50.0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 50.0}},{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};
  double FB[itr][legn][coor]= {{{0, 0, 0}, {0, -40, 0}, {0, 0, 0}, {0, 40, 0}}, {{0, 60, 0}, {0, -60, 0}, {0, -20, 0}, {0, 20, 0}}, {{0, 40, 0}, {0, 0, 0}, {0, -40, 0}, {0, 0, 0}}, {{0, 20, 0}, {0, 60, 0}, {0, -60, 0}, {0, -20, 0}}, {{0, 0, 0}, {0, 40, 0}, {0, 0, 0}, {0, -40, 0}}, {{0, -20, 0}, {0, 20, 0}, {0, 60, 0}, {0, -60, 0}}, {{0, -40, 0}, {0, 0, 0}, {0, 40, 0}, {0, 0, 0}},{{0, -60, 0}, {0, -20, 0}, {0, 20, 0}, {0, 60, 0}}};
  double LR[itr][legn][coor]= {{{0, 0, 0}, {-40, 0, 0}, {0, 0, 0}, {40, 0, 0}}, {{60, 0, 0}, {-60, 0, 0}, {-20, 0, 0}, {20, 0, 0}}, {{40, 0, 0}, {0, 0, 0}, {-40, 0, 0}, {0, 0, 0}}, {{20, 0, 0}, {60, 0, 0}, {-60, 0, 0}, {-20, 0, 0}}, {{0, 0, 0}, {40, 0, 0}, {0, 0, 0}, {-40, 0, 0}}, {{-20, 0, 0}, {20, 0, 0}, {60, 0, 0}, {-60, 0, 0}}, {{-40, 0, 0}, {0, 0, 0}, {40, 0, 0}, {0, 0, 0}},{{-60, 0, 0}, {-20, 0, 0}, {20, 0, 0}, {60, 0, 0}}};
  double RT[itr][legn][coor]= {{{0.0, 0.0, 0}, {19.98, -13.05, 0}, {0.0, 0.0, 0}, {-18.33, -15.28, 0}}, {{26.8, -23.69, 0}, {30.51, -18.68, 0}, {-9.8, -6.81, 0}, {-9.39, -7.37, 0}}, {{18.33, -15.28, 0}, {0.0, 0.0, 0}, {-19.98, -13.05, 0}, {0.0, 0.0, 0}}, {{9.39, -7.37, 0}, {-26.8, 23.69, 0}, {-30.51, -18.68, 0}, {9.8, 6.81, 0}}, {{0.0, 0.0, 0}, {-18.33, 15.28, 0}, {0.0, 0.0, 0}, {19.98, 13.05, 0}}, {{-9.8, 6.81, 0}, {-9.39, 7.37, 0}, {26.8, 23.69, 0}, {30.51, 18.68, 0}}, {{-19.98, 13.05, 0}, {0.0, 0.0, 0}, {18.33, 15.28, 0}, {0.0, 0.0, 0}},{{-30.51, 18.68, 0}, {9.8, -6.81, 0}, {9.39, 7.37, 0}, {-26.8, -23.69, 0}}};
};
*/

struct STRUCT { //defining struct called test_packet
  float item1;  // LR
  float item2;  // FB
  float item3;
  float item4;
  int item5;
  int item6;
  int item7;
  int item8;
  int item9;
  int item10;
} packet;

#endif
