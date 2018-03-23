# PID Controller project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview

PID stands for Proportional-Integral-Derivative. These three controllers are combined in such a way that it produces a control signal. This is how the vehicle uses steering, throttle, and brake to move through the world, executing a trajectory created by the path planning block.

In this project we will revisit the lake race track from the Behavioral Cloning Project. This time, however, we will implement a PID controller in C++ to maneuver the vehicle around the track! The simulator will provide the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

## Implementation

This is very basic controller in control theory and finding proportional, integral, and derivative componets using CTE to findout the control signal. 

```
cte = robot.y
diff_cte = cte - prev_cte
prev_cte = cte
int_cte += cte
steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte

```
This is the pseudocode to findout the steering value based PID controller algorithm.

PID project code to findout steering value based on the CTE:

```
void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
}

double PID::TotalError() {
    return (-Kp * p_error) - (Ki * i_error) - (Kd * d_error);
}

void PID::Twiddle() {

    
}

//cte is the cross track error fed into the controller from a sensor.
//p_error, i_error, and d_error are proportional, integral and derivativecomponents’ errors respectively.
//Kp, Ki, and Kd are those three parameters to be optimized.

```

These functioned get called from main.cpp.

The intial value Kp, Ki, Kd I selected using trail and error method. Then use twiddle to optimise further. 

Intial Value I used is 
```
 double p[3] = {0.05, 0.0001, 1.5};
 double dp[3] = {.01, .0001, .1};
``
Then use Twiddle to optimise the p values and got the below optimised values for p:

```
0.06, 0.00031, 1.29
```

Once I found the the optimised value, set the twiddle value to false to run the car through the simulator.

```
bool twiddle = false;
```

Actual twiddle algorothm in python as follows:

```
def twiddle(tol=0.2): 
    p = [0, 0, 0]
    dp = [1, 1, 1]
    robot = make_robot()
    x_trajectory, y_trajectory, best_err = run(robot, p)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}".format(it, best_err))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = make_robot()
            x_trajectory, y_trajectory, err = run(robot, p)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                robot = make_robot()
                x_trajectory, y_trajectory, err = run(robot, p)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p

```

And I had to modify it for this project and code as follows:
```
if (twiddle == true){
    total_cte = total_cte + pow(cte,2);
    if(n==0){

      pid.Init(p[0],p[1],p[2]); 
    }
    //Steering value
    pid.UpdateError(cte);
    steer_value = pid.TotalError();

    // DEBUG
    //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << " Count: " << n << std::endl;
    n = n+1;
    if (n > max_n){ 


      //double sump = p[0]+p[1]+p[2];
      //std::cout << "sump: " << sump << " ";
      if(first == true) {
        std::cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
        p[p_iterator] += dp[p_iterator];
        //pid.Init(p[0], p[1], p[2]);
        first = false;
      }else{
        error = total_cte/max_n;

        if(error < best_error && second == true) {
            best_error = error;
            best_p[0] = p[0];
            best_p[1] = p[1];
            best_p[2] = p[2];
            dp[p_iterator] *= 1.1;
            sub_move += 1;
            std::cout << "iteration: " << total_iterator << " ";
            std::cout << "p_iterator: " << p_iterator << " ";
            std::cout << "p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
            std::cout << "error: " << error << " ";
            std::cout << "best_error: " << best_error << " ";
            std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
        }else{
          //std::cout << "else: ";
          if(second == true) {
            std::cout << "Intermediate p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
            p[p_iterator] -= 2 * dp[p_iterator];
            //pid.Init(p[0], p[1], p[2]);
            second = false;
          }else {
            std::cout << "iteration: " << total_iterator << " ";
            std::cout << "p_iterator: " << p_iterator << " ";
            std::cout << "p[0] p[1] p[2]: " << p[0] << " " << p[1] << " " << p[2] << " ";
            if(error < best_error) {
                best_error = error;
                best_p[0] = p[0];
                best_p[1] = p[1];
                best_p[2] = p[2];
                dp[p_iterator] *= 1.1;
                sub_move += 1;
            }else {
                p[p_iterator] += dp[p_iterator];
                dp[p_iterator] *= 0.9;
                sub_move += 1;
            }
            std::cout << "error: " << error << " ";
            std::cout << "best_error: " << best_error << " ";
            std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
          }
        }

      }


      if(sub_move > 0) {
        p_iterator = p_iterator+1;
        first = true;
        second = true;
        sub_move = 0;
      }
      if(p_iterator == 3) {
        p_iterator = 0;
      }
      total_cte = 0.0;
      n = 0;
      total_iterator = total_iterator+1;

      double sumdp = dp[0]+dp[1]+dp[2];
      if(sumdp < tol) {
        //pid.Init(p[0], p[1], p[2]);
        std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << best_p[1] << best_p[2] << " ";
        //ws.close();
        //std::cout << "Disconnected" << std::endl;
      } else {
        std::string reset_msg = "42[\"reset\",{}]";
        ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
      }

    } else {
      msgJson["steering_angle"] = steer_value;
      msgJson["throttle"] = throttle_value;
      auto msg = "42[\"steer\"," + msgJson.dump() + "]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }

  } 
```
