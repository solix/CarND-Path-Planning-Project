bool manoeuvre_safe = false;
int cost = 100;
State current_state;
State next_state;
current_state = Lane_Keep;
bool best_lane_right = false;




======================
//frenet is a great help
if (prev_size > 3) {
car_s = end_path_s;
}
bool illegal_distance = false;
int closest_distance = 80000;


for (int i = 0; i < sensor_fusion.size(); i++)
{
int other_car_id = sensor_fusion[i][0];
double other_car_x = sensor_fusion[i][1];
double other_car_y = sensor_fusion[i][2];
double other_car_vx = sensor_fusion[i][3];
double other_car_vy = sensor_fusion[i][4];
float other_car_s = sensor_fusion[i][5];
float other_car_d = sensor_fusion[i][6];


//if the car is in the same lane as our car then we
//are interested to control car speed and distance
if (other_car_d < (2 + 4 * lane + 2) && other_car_d > (2 + 4 * lane - 2) )
{

//predict where the other car will be in the future
double other_car_speed = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);

other_car_s += ((double) prev_size * 0.02 * other_car_speed);

int distance_to_front_car = (other_car_s - car_s);

//if we are approaching a car in front then do some action
if ((other_car_s > car_s) && ((other_car_s - car_s) < 30) && distance_to_front_car < closest_distance)
{
closest_distance = distance_to_front_car;

if (closest_distance > 10) {
if(ref_vel  > other_car_speed){
ref_vel -=.137;
cout<<"car vs other speed"<< ref_vel << ","<< other_car_speed<<endl;
}

if(closest_distance>10 && closest_distance < 20){
ref_vel -= other_car_speed * 2.37;
}

}

illegal_distance = true;



}

}
}



if (ref_vel < 49.5 ) {
ref_vel += .224;
}

===========================

          if (illegal_distance) {

            ref_vel -= .224;

            
            // cout << "state changed to ---> " << current_state << endl;
            float gap = 0;
            manoeuvre_safe = false;
            bool lane_safe = true;
            bool take_over = false;
            for (int i = 0; i < sensor_fusion.size(); i++)
            {
              float other_car_d = sensor_fusion[i][6];


              // cout << "preparing lane change" << endl;


              if (current_state == Lane_Keep ) {

              if (lane != 0) {
                next_state = Prep_LCL;
              } else if (lane != 2) {
                next_state = Prep_LCR;
              } 
              //transition
            current_state = next_state;
             }
              bool left_lane_safe = true;
              if (current_state == Prep_LCL ) {
                cout << "looking at the left lane" << endl;


                if ((other_car_d < (2 + 4 * (lane - 1) + 2)) && (other_car_d > (2 + 4 * (lane - 1) - 2)) ) {
                  double other_car_vx = sensor_fusion[i][3];
                  double other_car_vy = sensor_fusion[i][4];
                  float other_car_s = sensor_fusion[i][5];
                  double other_car_speed = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);
                  other_car_s += ((double) prev_size * 0.02 * other_car_speed);
                  double dist_s = other_car_s - car_s;

                  if (dist_s < 45 && dist_s > -15) {
                    cout << "changing state LCL not safe" << endl;
                    lane_safe = false;
                    if(lane == 1){
                      current_state = Lane_Keep;
                      next_state = Prep_LCR;
                      left_lane_safe = false;
                      current_state = next_state;
                      cout << "switching state PREP CLR " << endl;
                    }


                  }

                  if (lane_safe && abs(dist_s) > 32 ) {
                    manoeuvre_safe = true;
                    cout << "changing state LCL safe" << endl;
                    if(!take_over){
                    current_state = LCL;
                    left_lane_safe = true;
                  }
                  } 

                  cout << "left lane gap: " << dist_s << endl;

                }
              } else if (current_state == Prep_LCR) {

                if ((other_car_d < (2 + 4 * (lane + 1) + 2)) && (other_car_d > (2 + 4 * (lane + 1) - 2)) ) {
                  cout << "Looking a righ lane " << endl;
                  double other_car_vx = sensor_fusion[i][3];
                  double other_car_vy = sensor_fusion[i][4];
                  float other_car_s = sensor_fusion[i][5];
                  float other_car_d = sensor_fusion[i][6];
                  double other_car_speed = sqrt(other_car_vx * other_car_vx + other_car_vy * other_car_vy);
                  other_car_s += ((double) prev_size * 0.02 * other_car_speed);

                  double dist_s = other_car_s - car_s;

                  if (dist_s < 45 && dist_s > -15) {
                    cout << "changing state LCR not safe" << endl;
                    lane_safe = false;
                    if(lane == 1 && !left_lane_safe){
                      current_state = Lane_Keep;
                      next_state = Prep_LCL;
                      current_state = next_state;
                      cout << "switching state PREP LCL " << endl;
                    }

                  }

                  if (lane_safe && abs(dist_s) > 32  ) {
                    manoeuvre_safe = true;
                    cout << "changing state LCR safe" << endl;
                    if(!take_over){
                    current_state = LCR;
                    left_lane_safe = true;
                    }

                  } 
                  cout << "right lane gap: " << dist_s << endl;

                }
              }



              




              
                   


            }
     
            
              if ( manoeuvre_safe) {
                cout << "manoeuvre_safe with closest distance " << closest_distance<< endl;
                if (current_state == LCL) {
                  lane -= 1;
                  take_over =true;
                  // cout << "lane keep mode!" << endl;
                  next_state = Lane_Keep;
                  // left_lane_safe = true;
                }
                if (current_state == LCR) {
                  lane += 1;
                  take_over = true;
                  // cout << "lane keep mode!" << endl;
                  next_state = Lane_Keep;
                  // left_lane_safe = true;
                }
                manoeuvre_safe = false;
                current_state = next_state;
              }
          }