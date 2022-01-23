#include <math.h>
int factorial(int n){
  if (n == 1 || n == 0){
    return 1;
  }
  else{
    return n * factorial(n - 1);
  }
}

int choose(int n, int k) {
  return factorial(n) / (factorial(k) * factorial(n - k));
}


float bezArray[12][12] = {
  {1,0,0,0,0,0,0,0,0,0,0,0},
  {1,1,0,0,0,0,0,0,0,0,0,0},
  {1,2,1,0,0,0,0,0,0,0,0,0},
  {1,3,3,1,0,0,0,0,0,0,0,0},
  {1,4,6,4,1,0,0,0,0,0,0,0},
  {1,5,10,10,5,1,0,0,0,0,0,0},
  {1,6,15,20,15,6,1,0,0,0,0,0},
  {1,7,21,35,35,21,7,1,0,0,0,0},
  {1,8,28,56,70,56,28,8,1,0,0,0},
  {1, 9, 36, 84, 126, 126, 84, 36, 9, 1,0,0},
  {1, 10, 45, 120, 210, 252, 210, 120, 45, 10, 1,0},
  {1, 11, 55, 165, 330, 462, 462, 330, 165, 55, 11, 1}
};

void bezier(float t, float control_points[][2], size_t len, float traj[2]) {
  float x = 0;
  float y = 0;
  float bez = 0;
  for(size_t i = 0; i < len; i++)  {
//    bez = choose(len - 1, (int)i) * pow((1 - t),(len - 1 - (int)i)) * pow(t,(int)i);
    bez = bezArray[len-1][i] * pow((1 - t),(len - 1 - (int)i)) * pow(t,(int)i);
    x += bez * control_points[i][0];
    y += bez * control_points[i][1];
  }
//  Serial.printf("Setting Position (%f,%f)\n",x,y);
  traj[0] = x;
  traj[1] = y;
}
