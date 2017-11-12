# **Model Predictive Controller Project**

## Goals

The aim of the project is to complete a whole track with the aid of the simulator. 

For the first time we are using the IPOPT and CPPAD libraries that will help to make derivatives and predict the trayectories of the vehicle.

The code takes a small time step "dt" and based on it, the system calculates the next step from the trajectory. 

##  Implemented model

There are some kinematic equations that models the next system step. 

fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0/Lf * delta * dt);
      fg[1 + v_start + t] = v1 - (v0 + a * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta * dt);

Those takes into account the previous situation to model the new one. 

## Chosen N and dt

I chose these values empirically, my proposal is N = 10 and dt = 0.1

## Waypoints

I first transform the waypoints to the vehicle perspective. This shifts everything to the origin (0,0)

## Latency

The latency of the system sums up until 100ms, note that the time step of the system is exactly the same (dt = 0.1). This makes the system to jump one time step. 
To solve this problem I'm using this loop 

if (t > 1) {  
        a = vars[a_start + t - 2];
        delta = vars[delta_start + t - 2];
      }



