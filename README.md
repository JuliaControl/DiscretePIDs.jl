# DiscretePIDs

[![Build Status](https://github.com/JuliaControl/DiscretePIDs.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/JuliaControl/DiscretePIDs.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/JuliaControl/DiscretePIDs.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/JuliaControl/DiscretePIDs.jl)


This package implements a discrete-time PID controller on the form
$$U(s) = K \left( bR(s) - Y(s) + \dfrac{1}{sT_i} \left( R(s) - Y(s) \right) - \dfrac{sT_d}{1 + s T_d / N}Y(s) \right) + U_\textrm{ff}(s)$$

where
- $u(t) \leftrightarrow U(s)$ is the control signal
- $y(t) \leftrightarrow Y(s)$ is the measurement signal
- $r(t) \leftrightarrow R(s)$ is the reference / set point
- $u_\textrm{ff}(t) \leftrightarrow U_\textrm{ff}(s)$ is the feed-forward contribution
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

The numeric type used by the controller (the `T` in `DiscretePID{T}`) is determined by the types of the parameters. To use a custom number type, e.g., a fixed-point number type, simply pass the parameters as that type, see example below. The controller will automatically convert measurements and references to this type before performing the control calculations.

## Example using ControlSystems:
The following example simulates the PID controller using ControlSystems.jl. We will simulate a load disturbance $d(t) = 1$ entering on the process input, while the reference is $r(t) = 0$.

```julia
using DiscretePIDs, ControlSystemsBase, Plots
Tf = 15   # Simulation time
K  = 1    # Proportional gain
Ti = 1    # Integral time
Td = 1    # Derivative time
Ts = 0.01 # sample time

P   = c2d(ss(tf(1, [1, 1])), Ts) # Process to be controlled, discretized using zero-order hold
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

## Example using DifferentialEquations:
The following example is identical to the one above, but uses DifferentialEquations.jl to simulate the PID controller. This is useful if you want to simulate the controller in a more complex system, e.g., with a nonlinear plant.

There are several different ways one could go about including a discrete-time controller in a continuous-time simulation, in particular, we must choose a way to store the computed control signal
1. Use a global variable into which we write the control signal at each discrete time step.
2. Add an extra state variable to the system, and use this state to store the control signal. This is the approach taken in the example below since it has the added benefit of adding the computed control signal to the solution object.

We will use a `DiffEqCallbacks.PeriodicCallback` in which we perform the PID-controller update, and store the computed control signal in the extra state variable.

```julia
using DiscretePIDs, ControlSystemsBase, OrdinaryDiffEq, DiffEqCallbacks, Plots

Tf = 15   # Simulation time
K  = 1    # Proportional gain
Ti = 1    # Integral time
Td = 1    # Derivative time
Ts = 0.01 # sample time

P = ss(tf(1, [1, 1]))  # Process to be controlled in continuous time
A, B, C, D = ssdata(P) # Extract the system matrices
pid = DiscretePID(; K, Ts, Ti, Td)

function dynamics!(dxu, xu, p, t)
    A, B, C, r, d = p   # We store the reference and disturbance in the parameter object
    x = xu[1:P.nx]      # Extract the state
    u = xu[P.nx+1:end]  # Extract the control signal
    dxu[1:P.nx] .= A*x .+ B*(u .+ d) # Plant input is control signal + disturbance
    dxu[P.nx+1:end] .= 0             # The control signal has no dynamics, it's updated by the callback
end

cb = PeriodicCallback(Ts) do integrator
    p = integrator.p    # Extract the parameter object from the integrator
    (; C, r, d) = p     # Extract the reference and disturbance from the parameter object
    x = integrator.u[1:P.nx] # Extract the state (the integrator uses the variable name `u` to refer to the state, in control theory we typically use the variable name `x`)
    y = (C*x)[]         # Simulated measurement
    u = pid(r, y)       # Compute the control signal
    integrator.u[P.nx+1:end] .= u # Update the control-signal state variable 
end

parameters = (; A, B, C, r=0, d=1) # reference = 0, disturbance = 1
xu0 = zeros(P.nx + P.nu) # Initial state of the system + control signals
prob = ODEProblem(dynamics!, xu0, (0, Tf), parameters, callback=cb) # reference = 0, disturbance = 1
sol = solve(prob, Tsit5(), saveat=Ts)

plot(sol, layout=(2, 1), ylabel=["x" "u"], lab="")
```
The figure should look more or less identical to the one above, except that we plot the control signal $u$ instead of the combined input $u + d$ like we did above. Due to the fast sample rate $T_s$, the control signal looks continuous, however, increase $T_s$ and you'll notice the zero-order-hold nature of $u$.

## Details
- The derivative term only acts on the (filtered) measurement and not the command signal. It is thus safe to pass step changes in the reference to the controller. The parameter $b$ can further be set to zero to avoid step changes in the control signal in response to step changes in the reference.
- Bumpless transfer when updating `K` is realized by updating the state `I`. See the docs for `set_K!` for more details.
- The total control signal $u(t)$ (PID + feed-forward) is limited by the integral anti-windup.

## Simulation of fixed-point arithmetic
If the controller is ultimately to be implemented on a platform without floating-point hardware, you can simulate how it will behave with fixed-point arithmetics using the `FixedPointNumbers` package. The following example modifies the first example above and shows how to simulate the controller using 16-bit fixed-point arithmetics with 10 bits for the fractional part:
```julia
using FixedPointNumbers
T = Fixed{Int16, 10} # 16-bit fixed-point with 10 bits for the fractional part
pid = DiscretePID(; K = T(K), Ts = T(Ts), Ti = T(Ti), Td = T(Td))
res_fp = lsim(P, ctrl, Tf)
plot([res, res_fp], plotu=true, lab=["Float64" "" string(T) ""]); ylabel!("u + d", sp=2)
```
![Fixed-point simulation result](https://user-images.githubusercontent.com/3797491/249732319-0a3890d5-cb9c-45c2-93c7-20d3c7db0cf2.png)

The fixed-point controller behaves roughly the same in this case, but artifacts are clearly visible. If the number of bits used for the fractional part is decreased, the controller will start to misbehave.

## See also
- [TrajectoryLimiters.jl](https://github.com/baggepinnen/TrajectoryLimiters.jl) To generate dynamically feasible reference trajectories with bounded velocity and acceleration given an instantaneous reference $r(t)$ which may change abruptly.
- [SymbolicControlSystems.jl](https://github.com/JuliaControl/SymbolicControlSystems.jl) For C-code generation of LTI systems.