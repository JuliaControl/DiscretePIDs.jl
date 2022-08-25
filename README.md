# DiscretePIDs

[![Build Status](https://github.com/JuliaControl/DiscretePIDs.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/JuliaControl/DiscretePIDs.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/JuliaControl/DiscretePIDs.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/JuliaControl/DiscretePIDs.jl)


This package implements a discrete-time PID controller on the form
$$U(s) = K \left( bR(s) - Y(s) + \dfrac{1}{sT_i} \left( R(s) - Y(s) \right) - \dfrac{sT_d}{1 + s T_d / N}Y(s) \right) + u_{ff}(s)$$

where
- $u(t) \leftrightarrow U(s)$ is the control signal
- $y(t) \leftrightarrow Y(s)$ is the measurement signal
- $r(t) \leftrightarrow R(s)$ is the reference / set point
- $u_{ff}(t) \leftrightarrow u_{ff}(s)$ is the feed-forward contribution
- $K$ is the proportional gain
- $T_i$ is the integral time
- $T_d$ is the derivative time
- $N$ is the maximum derivative gain
- $b \in [0, 1]$ is the proportion of the reference signal that appears in the proportional term.

The controller further has output saturation controlled by `umin, umax` and integrator anti-windup controlled by the tracking time $T_t$.

Construct a controller using
```julia
pid = DiscretePID(; K = 1, Ti = false, Td = false, Tt = √(Ti*Td), N = 10, b = 1, umin = -Inf, umax = Inf, Ts, I = 0, D = 0, yold = 0)
```
and compute a control signal using
```julia
u = pid(r, y, uff)
```
or
```julia
u = calculate_control!(pid, r, y, uff)
```

The parameters $K, T_i, T_d$ may be updated using the functions, `set_K!, set_Ti!, set_Td!`.

## Example:
The following example simulates the PID controller using ControlSystems.jl. We will simulate a load disturbance $d(t) = 1$ entering on the process input, while the reference is $r(t) = 0$.

```julia
using DiscretePIDs, ControlSystems, Plots
Tf = 15 # Simulation time
K  = 1
Ti = 1
Td = 1
Ts = 0.01 # sample time

P   = c2d(ss(tf(1, [1, 1])), Ts) # Process to be controlled
pid = DiscretePID(; K, Ts, Ti, Td)

ctrl = function(x,t)
    y = (P.C*x)[] # measurement
    d = 1         # disturbance
    r = 0         # reference
    u = pid(r, y)
    u + d # Plant input is control signal + disturbance
end

res = lsim(P, ctrl, Tf)

plot(res, plotu=true); ylabel!("u + d", sp=2)
```
![Simulation result](https://user-images.githubusercontent.com/3797491/172366365-c1533aed-e877-499d-9ebb-01df62107dfb.png)

## Details
- The derivative term only acts on the (filtered) measurement and not the command signal. It is thus safe to pass step changes in the reference to the controller. The parameter $b$ can further be set to zero to avoid step changes in the control signal in response to step changes in the reference.
- Bumpless transfer when updating `K` is realized by updating the state `I`. See the docs for `set_K!` for more details.
- The total control signal $u(t)$ (PID + feed-forward) is limited by the integral anti-windup.
