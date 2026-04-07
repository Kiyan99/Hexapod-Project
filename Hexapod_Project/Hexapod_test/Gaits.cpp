#include "Gaits.h"
#include <math.h>

Gaits::Gaits(){

}

void Gaits::down_up(){

  // Sitting down
  if(standing == true){
    for (float t = 0; t < 1.0; t += 0.05){

      int x = global_x + (170 - global_x) * t;
      int y = global_y + (0 - global_y) * t;

      for (int i = 0; i < 6; i++){
        computeIK(*allLegs[i], x, y, 0);
        moveLeg(*allLegs[i]);
      }
    }
    standing = false;

  } 

  // Standing up
  else if(standing == false){
    for (float t = 0; t < 1.0; t += 0.05){

      int x = 170 + (global_x - 170) * t;
      int y = 0 + (global_y - 0) * t;

      for (int i = 0; i < 6; i++){
        computeIK(*allLegs[i], x, y, 0);
        moveLeg(*allLegs[i]);
      }
    }

    // Leg adjustment legs 1, 2, 3
    for (float t = 0; t < 1.0; t += 0.1){

    int y = global_y + (-80 - global_y) * t;

      for (int i = 0; i < 3; i++) {
        computeIK(*allLegs[i], global_x, y, 0);
        moveLeg(*allLegs[i]);
      }
    }


    for (float t = 0; t < 1.0; t += 0.1){

        int y = -80 + (global_y - (-80)) * t;

      for (int i = 0; i < 3; i++) {
        computeIK(*allLegs[i], global_x, y, 0);
        moveLeg(*allLegs[i]);
      }
    }


    // Leg adjustment legs 4, 5, 6
    for (float t = 0; t < 1.0; t += 0.1){

    int y = global_y + (-80 - global_y) * t;

      for (int i = 3; i < 6; i++) {
        computeIK(*allLegs[i], global_x, y, 0);
        moveLeg(*allLegs[i]);
      }
    }


    for (float t = 0; t < 1.0; t += 0.1){

        int y = -80 + (global_y - (-80)) * t;

      for (int i = 3; i < 6; i++) {
        computeIK(*allLegs[i], global_x, y, 0);
        moveLeg(*allLegs[i]);
      }
    }    
    
    standing = true;

  }

}



void Gaits::reset_gaits(){
  phase = 1;
  t = 0.0;

  if (standing == true){
    for (int i = 0; i < 6; i++) {
      computeIK(*allLegs[i], global_x, global_y, 0);
      moveLeg(*allLegs[i]);
    }
  }
  delay(100);


}



void Gaits::tripod_forward() {

  if (standing == true){
    // Phase 1
    if (phase == 1){
      // Moving forward
      int y = global_y + arc * sin(M_PI * t);       // Lift arc


      // Front legs
      // Pulling and pushing values used by front and read legs
      int z_push = (1 - t) * 0 + t * 30;
      int z_pull = (1 - t) * 30;  
      int x_push = (1 - t) * 110 + t * global_x;
      int x_pull = (1 - t) * global_x + t * 110;  
      computeIK(leg1, x_push, y, z_push);
      moveLeg(leg1);

      computeIK(leg4, x_pull, global_y, -z_pull);
      moveLeg(leg4);

      // Middle legs
      // Pulling back for middle legs
      int z = (1 - t) * -15 + t * 15; 
      int z_mid_pull = (1 - t) * 15 + t * -15;
      computeIK(leg2, global_x, y, -z);
      moveLeg(leg2); 

      computeIK(leg5, global_x, global_y, -z);  
      moveLeg(leg5);     


      // Rear legs    
      computeIK(leg3, x_pull, y, -z_pull);
      moveLeg(leg3);

      computeIK(leg6, x_push, global_y, z_push);
      moveLeg(leg6);    

    }

    // Phase 2
    else if (phase == 2){
        
      // Moving forward
      int y = global_y + arc * sin(M_PI * t);           // Lift arc


      //front Legs
      int z_push = (1 - t) * 0 + t * 30;
      int z_pull = (1 - t) * 30;  
      int x_push = (1 - t) * 110 + t * global_x;
      int x_pull = (1 - t) * global_x + t * 110;  
      computeIK(leg4, x_push, y, -z_push);
      moveLeg(leg4);

      computeIK(leg1, x_pull, global_y, z_pull);
      moveLeg(leg1);



      // Midlle Legs
      // Pulling back
      int z = (1 - t) * -15 + t * 15;
      int z_mid_pull = (1 - t) * 15 + t * -15;
      computeIK(leg5, global_x, y, z);
      moveLeg(leg5);

      computeIK(leg2, global_x, global_y, z);
      moveLeg(leg2);



      // Rear legs
      computeIK(leg6, x_pull, y, z_pull);
      moveLeg(leg6);

      computeIK(leg3, x_push, global_y, -z_push);
      moveLeg(leg3);


    }

    t = t + 0.1;

    if (t > 1.0){
      t = 0.0;

      if (phase == 1){
      phase = 2;
      }
      else{
        phase = 1;
      }
    }
  }
}







void Gaits::tripod_revers(){
  // In this function legs are reversed, it is bassically walking forward with
  // front and rear legs being swapped
  // Phase 1

  if (standing == true){
    // for-loop will run 10 timmes
    if (phase == 1) {
      
      
      int y = global_y + arc * sin(M_PI * t);           // Lift arc


      // Front legs
      // Pulling and pushing values used by front legs
      int z_push = (1 - t) * 0 + t * 30;
      int z_pull = (1 - t) * 30;  
      int x_push = (1 - t) * 110 + t * global_x;
      int x_pull = (1 - t) * global_x + t * 110;  
      computeIK(leg6, x_push, y, z_push);
      moveLeg(leg6);

      computeIK(leg3, x_pull, global_y, -z_pull);
      moveLeg(leg3);

      // Middle legs
      // Pulling back for middle legs
      int z = (1 - t) * -15 + t * 15; 
      computeIK(leg5, global_x, y, -z);
      moveLeg(leg5); 

      computeIK(leg2, global_x, global_y, -z);  
      moveLeg(leg2);     


      // Rear legs    
      computeIK(leg4, x_pull, y, -z_pull);
      moveLeg(leg4);

      computeIK(leg1, x_push, global_y, z_push);
      moveLeg(leg1);    


    }


    // Phase 2
    else if (phase == 2) {
      
      // Moving forward
                    // Forward swing
      int y = global_y + arc * sin(M_PI * t);           // Lift arc


      //front Legs
      int z_push = (1 - t) * 0 + t * 30;
      int z_pull = (1 - t) * 30;  
      int x_push = (1 - t) * 110 + t * global_x;
      int x_pull = (1 - t) * global_x + t * 110;  
      computeIK(leg6, x_pull, global_y, z_pull);
      moveLeg(leg6);

      computeIK(leg3, x_push, y, -z_push);
      moveLeg(leg3);



      // Midlle Legs
      // Pulling back
      int z = (1 - t) * -15 + t * 15;
      computeIK(leg5, global_x, global_y, z);
      moveLeg(leg5);

      computeIK(leg2, global_x, y, z);
      moveLeg(leg2);



      // Rear legs
      computeIK(leg4, x_push, global_y, -z_push);
      moveLeg(leg4);

      computeIK(leg1, x_pull, y, z_pull);
      moveLeg(leg1);
      

    }


    t = t + 0.1;

    if (t > 1.0){
      t = 0.0;

      if (phase == 1){
      phase = 2;
      }
      else{
        phase = 1;
      }
    }
  }
}



void Gaits::turn_right(){


  if (standing == true){
    // Phase 1
    if (phase == 1) {

      int z = (1 - t) * -30 + t * 30;              // Forward swing
      int y = global_y + arc * sin(M_PI * t);           // Lift arc      

      computeIK(leg1, global_x, y, z);
      moveLeg(leg1);

      computeIK(leg2, global_x, y, z);
      moveLeg(leg2);

      computeIK(leg3, global_x, y, z);
      moveLeg(leg3);


      int z_pull = (1 - t) * 30 + t * -30;
      computeIK(leg4, global_x, global_y, z_pull);
      moveLeg(leg4);

      computeIK(leg5, global_x, global_y, z_pull);
      moveLeg(leg5);

      computeIK(leg6, global_x, global_y, z_pull);
      moveLeg(leg6);


    }


    // Phase 2
    else if (phase == 2) {

      int z = (1 - t) * -30 + t * 30;              // Forward swing
      int y = global_y + arc * sin(M_PI * t);           // Lift arc      

      computeIK(leg4, global_x, y, z);
      moveLeg(leg4);

      computeIK(leg5, global_x, y, z);
      moveLeg(leg5);

      computeIK(leg6, global_x, y, z);
      moveLeg(leg6);


      int z_pull = (1 - t) * 30 + t * -30;
      computeIK(leg1, global_x, global_y, z_pull);
      moveLeg(leg1);

      computeIK(leg2, global_x, global_y, z_pull);
      moveLeg(leg2);

      computeIK(leg3, global_x, global_y, z_pull);
      moveLeg(leg3);

    }

    t = t + 0.1;

    if (t > 1.0){
      t = 0.0;

      if (phase == 1){
      phase = 2;
      }
      else{
        phase = 1;
      }
    }
  }
}


void Gaits::turn_left(){

  if (standing == true){
    // Phase 1
    if(phase == 1) {

      int z = (1 - t) * 30 + t * -30;              // Forward swing
      int y = global_y + arc * sin(M_PI * t);           // Lift arc      

      computeIK(leg1, global_x, y, z);
      moveLeg(leg1);

      computeIK(leg2, global_x, y, z);
      moveLeg(leg2);

      computeIK(leg3, global_x, y, z);
      moveLeg(leg3);


      int z_pull = (1 - t) * -30 + t * 30;
      computeIK(leg4, global_x, global_y, z_pull);
      moveLeg(leg4);

      computeIK(leg5, global_x, global_y, z_pull);
      moveLeg(leg5);

      computeIK(leg6, global_x, global_y, z_pull);
      moveLeg(leg6);


    }


    // Phase 2
    else if (phase == 2) {

      int z = (1 - t) * 30 + t * -30;              // Forward swing
      int y = global_y + arc * sin(M_PI * t);           // Lift arc      

      computeIK(leg4, global_x, y, z);
      moveLeg(leg4);

      computeIK(leg5, global_x, y, z);
      moveLeg(leg5);

      computeIK(leg6, global_x, y, z);
      moveLeg(leg6);


      int z_pull = (1 - t) * -30 + t * 30;
      computeIK(leg1, global_x, global_y, z_pull);
      moveLeg(leg1);

      computeIK(leg2, global_x, global_y, z_pull);
      moveLeg(leg2);

      computeIK(leg3, global_x, global_y, z_pull);
      moveLeg(leg3);


    }

    t = t + 0.1;

    if (t > 1.0){
      t = 0.0;

      if (phase == 1){
      phase = 2;
      }
      else{
        phase = 1;
      }
    }
  }
}


void Gaits::crab_walk_right(){

  if (standing == true){
    // Phase 1
    if (phase == 1) {

      int z = (1 - t) * 0 + t * 30;              // Forward swing
      int y = global_y + arc * sin(M_PI * t);           // Lift arc      
      int x_push = (1 - t) * 110 + t * global_x; 
      int x_pull = (1 - t) * global_x + t * 110;


      computeIK(leg4, x_push, y, z);
      moveLeg(leg4);

      computeIK(leg5, x_pull, y, 0);
      moveLeg(leg5);

      computeIK(leg6, x_push, y, -z);
      moveLeg(leg6);


      computeIK(leg1, x_push, global_y, -z);
      moveLeg(leg1);

      computeIK(leg2, x_pull, global_y, 0);
      moveLeg(leg2);

      computeIK(leg3, x_push, global_y, z);
      moveLeg(leg3);

    }

    // Phase 2
    else if (phase == 2) {


      int y = global_y + arc * sin(M_PI * t);           // Lift arc      
      int x_push = (1 - t) * 110 + t * global_x; 
      int x_pull = (1 - t) * global_x + t * 110;     
      int z_pull = (1 - t) * 30 + t * 0;

      computeIK(leg4, x_pull, global_y, z_pull);
      moveLeg(leg4);

      computeIK(leg5, x_push, global_y, 0);
      moveLeg(leg5);

      computeIK(leg6, x_pull, global_y, -z_pull);
      moveLeg(leg6);


      computeIK(leg1, x_pull, y, -z_pull);
      moveLeg(leg1);
      
      computeIK(leg2, x_push, y, 0);
      moveLeg(leg2);

      computeIK(leg3, x_pull, y, z_pull);
      moveLeg(leg3);
      

    }

    t = t + 0.1;

    if (t > 1.0){
      t = 0.0;

      if (phase == 1){
      phase = 2;
      }
      else{
        phase = 1;
      }
    }
  }
}



void Gaits::crab_walk_left(){

  if (standing == true){
    // Phase 1
    if (phase == 1) {

      int z = (1 - t) * 0 + t * 30;              // Forward swing
      int y = global_y + arc * sin(M_PI * t);           // Lift arc      
      int x_push = (1 - t) * 110 + t * global_x; 
      int x_pull = (1 - t) * global_x + t * 110;

      computeIK(leg4, x_push, global_y, z);
      moveLeg(leg4);

      computeIK(leg5, x_pull, global_y, 0);
      moveLeg(leg5);

      computeIK(leg6, x_push, global_y, -z);
      moveLeg(leg6);



      computeIK(leg1, x_push, y, -z);
      moveLeg(leg1);

      computeIK(leg2, x_pull, y, 0);
      moveLeg(leg2);

      computeIK(leg3, x_push, y, z);
      moveLeg(leg3);


    }

    // Phase 2
    else if (phase == 2) {

      int y = global_y + arc * sin(M_PI * t);           // Lift arc      
      int x_push = (1 - t) * 110 + t * global_x; 
      int x_pull = (1 - t) * global_x + t * 110;     
      int z_pull = (1 - t) * 30 + t * 0;

      computeIK(leg4, x_pull, y, z_pull);
      moveLeg(leg4);

      computeIK(leg5, x_push, y, 0);
      moveLeg(leg5);

      computeIK(leg6, x_pull, y, -z_pull);
      moveLeg(leg6);



      computeIK(leg1, x_pull, global_y, -z_pull);
      moveLeg(leg1);

      computeIK(leg2, x_push, global_y, 0);
      moveLeg(leg2);

      computeIK(leg3, x_pull, global_y, z_pull);
      moveLeg(leg3);


    }

    t = t + 0.1;

    if (t > 1.0){
      t = 0.0;

      if (phase == 1){
      phase = 2;
      }
      else{
        phase = 1;
      }
    }
  }
}


