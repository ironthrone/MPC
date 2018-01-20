## Model
The model i use is kinematic car model,just as what the material is taught.  
There is six status variable : 
* px: car's x coordinate
* py: car's y coordinate
* psi: car's orientation angle
* v: car's speed
* cte: cross track error,the distance from car to middle line of road or reference trajectory
* epsi:orientation angle error, diff between car's orientaion angle and car's desire orientation controlled by reference trajectory  

and two actuators:
* delta: steering angle, produced by steering wheel, actuator
* a:accelerate, produced by car's throttle ,actuator

In every timestep,car runs at a constant speed and orientation.At timestep's end, modify car's speed and orientation according to accelerate and steer angle.

```
px_t1 = px_t + v_t * cos(psi_t) * dt;
py_t1 = px_t + v_t * sin(psi_t) * dt;
psi_t1 = psi_t + v_t/Lf * delta_t * dt;
v_t1 = v_t + a_t * dt;
cte_t1 = py_t - f(x_t) + v_t * sin(epsi) * dt;
epsi_t1 = psi_t - psides_t + v_t/Lf * delta * dt;

psides_t = atan(f'(x_t))
//Lf is a constant, means the distance from car's weight center to car's front wheel.In this project , Lf is gaven by the provider
//f(x) is the polynomial line/ reference trajectory/desired trajectory
```

## Choose N and dt
N * dt is horizon, it controls the car's predictive time range.It can neither be too large nor too small.  
If horizon is too large , disadvantage:
* there will be many error in the MPC predict,because my model is approximate
* calculation may speed more time, this is crucial in a real time system
* remote predict only have little influence to car's current control input

If horizon is too small, disadvantage:
* car can not have enough information to response to nearby road change

I tried N and dt at start period of my project's implementation,at that time ref speed is 40 km/h.
my cost function is simple,just
```
 for (int i = 0; i < N; i++) {
      fg[0] +=  CppAD::pow(vars[cte_start + i], 2);
      fg[0] +=  CppAD::pow(vars[epsi_start + i], 2);
      //impact average speed
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }
```
The first combination is dt=1,N = 8, this is a stupid choice,car's prediction just weird. Then i try to decrease dt,i just tried dt = 0.5 and dt = 0.1.It is my lucky,i got a reasonable value ,the car runs successfully.  
But i also want to try other values to understand the impact ,then i try to increase N to 15, the car runs badly again , i understand it is because the horizon is too large .Then i decrease the N to find a reasonable horizon, the final N just 7,8.All right ,i come back to original.
Then i try to decrease dt, i tried 0.08,0.05,0.01.The smaller dt, the car runs more jitter.From my understand,the reason is car change actuation too frequently. finally i chooose dt  = 0.08

## Polynomial fit and Preprocessing
Polynomial fitting function is provided, i think there is no more talk about its implementation. I  choise order = 3 just as the material taught.  
I think in this section i should talk about coordinator transform. In my car model, current's cte is calculated by equation `y_t - f(x_t)`, just considered the diff in y axis. This equation is reasonable if we look from current car's local coordinate system, because of we only predict a small horizon , car can only change orientation angle little,so the change in x axis is small in the horizon time range.Car's motion can be approximated to straight line, thus cte is distance in y axis.  
But if we use the global system, we will make mistake with this equation.There is a abviously example: If the car's running direction is just global coordinate system's y axis, the cte should be diff in x axis. so if we still use this equation to calculate cte, a mistak is maken. 
So before polynomial fit, i make waypoints transform to car's local coordinate system. The code is :
```
 Eigen::VectorXd x_array(ptsx.size());
 Eigen::VectorXd y_array(ptsx.size());
 for (unsigned int i = 0; i < ptsx.size(); i++) {
    double x_middle = ptsx[i] - px;
    double y_middle = ptsy[i] - py;
    double x_transformed = x_middle * std::cos(-psi) - y_middle * std::sin(-psi);
    double y_transformed = x_middle * std::sin(-psi) + y_middle * std::cos(-psi);

    x_array(i) = x_transformed;
    y_array(i) = y_transformed;
 }
```
## Latency
I approximately consider the car runs obey the same model in latency period,so the car model can be used in the latency period. With this model,I calculated the car's status after latency time, and use these new position, orientation angle,accelerate as current status ,and  calculate actuators just as normal procedure.
```
  int latency = 100;
  px = px + latency  * v * cos(psi) / 1000;
  py = py + latency  * v * sin(psi) / 1000;
  v = v + latency  * a / 1000;
  psi = psi + v / 2.67 * delta * latency / 1000;
```
After this status transform , all the folowing calculate procedure keep same. But i need to tune the weight of cost function.

## Cost Function
The wight tune for cost function is crutial to my project, show the function first:
```
for (int i = 0; i < N; i++) {
      fg[0] += (10 + i + 1) * CppAD::pow(vars[cte_start + i], 2);
      fg[0] += (10000 + 2500*(i + 1)) * CppAD::pow(vars[epsi_start + i], 2);
      //impact average speed
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

	//constraint accelarate and steering angle
    for (int i = 0; i < N - 1; i++) {
      //control jitter
      fg[0] += 10000 * CppAD::pow(vars[delta_start + i], 2);
      //impact max speed
      fg[0] += 12 * CppAD::pow(vars[a_start + i], 2);
    }

	//constraint accelarate and steering angle change between nearby dt
    for (int i = 0; i < N - 2; i++) {
      fg[0] += 1000 * CppAD::pow(vars[delta_start + i] - vars[delta_start + i - 1], 2);
      fg[0] += 12 * CppAD::pow(vars[a_start + i] - vars[a_start + i - 1], 2);
    }
	// TODO add curvature_to_speed factor to decrease speed at curve road
```
In this function i constrainted cte,epsi, prevent v from becoming zero, constrainted steer angle,accelerate, steer angle change and accelerate change between nearby dt.I conclude some points :
* when car's speed become large, we must control epsi,steering angle  ,steering angle change better, constraints their range, thus car can runs smooth
* when car runs fast, we can allow a relatively larger cte in car's nearby range,but the final cte is smaller. Thus when car run over a curve ,it will make a distance to center line advancely to pass through the curve. Same as epsi

