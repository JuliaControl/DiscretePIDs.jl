# DiscretePIDs

[![Build Status](https://github.com/JuliaControl/DiscretePIDs.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/JuliaControl/DiscretePIDs.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/JuliaControl/DiscretePIDs.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/JuliaControl/DiscretePIDs.jl)


This package implements a discrete-time PID controller as an approximation of the continuous-time PID controller given by
$$U(s) = K \left( bR(s) - Y(s) + \dfrac{1}{sT_i} \left( R(s) - Y(s) \right) - \dfrac{sT_d}{1 + s T_d / N}Y(s) \right) + U_\textrm{ff}(s),$$
where
- $U(s)$ is the Laplace transform of the *control variable* (also called *manipulated variable*) $u(t)$,
- $Y(s)$ is the Laplace transform of the *measured output variable* (also called *process variable*) $y(t)$,
- $R(s)$ is the Laplace transform of the *reference variable* (also called *set point*) $r(t)$,
- $U_\textrm{ff}(s)$ is the Laplace transform of the *feedforward* contribution to the control variable $u_\textrm{ff}(t)$, 
- $K$ is the *proportional gain*,
- $T_\mathrm{i}$ is the *integral time*,
- $T_\mathrm{d}$ is the *derivative time*,
- $N$ is a parameter that limits the gain of the derivative term at high frequencies, typically ranges from 2 to 20,
- $b \in [0, 1]$ is a parameter that gives the proportion of the reference signal that appears in the proportional term.

*Saturation* of the controller output is parameterized by $u_{\min}$ and $u_{\max}$, and the integrator *anti-windup* is parameterized by the tracking time $T_\mathrm{t}$.

## Usage

Construct a controller by
```julia
pid = DiscretePID(; K = 1, Ti = false, Td = false, Tt = √(Ti*Td), N = 10, b = 1, umin = -Inf, umax = Inf, Ts, I = 0, D = 0, yold = 0)
```
and compute the control (the controller output) at a given time using
```julia
u = pid(r, y, uff)
```
or
```julia
u = calculate_control!(pid, r, y, uff)
```

The parameters $K$, $T_\mathrm{i}$, and $T_\mathrm{d}$ may be updated using the functions `set_K!`, `set_Ti!`, and `set_Td!`, respectively.

The numeric type used by the controller (the `T` in `DiscretePID{T}`) is determined by the types of the parameters. To use a custom number type, e.g., a fixed-point number type, simply pass the parameters as that type, see example below. The controller will automatically convert measurements and references to this type before performing the control calculations.

The *internal state* of the controller can be reset to zero using the function `reset_state!(pid)`. If repeated simulations using the same controller object are performed, the state should be reset between simulations.

## Examples

### Example using ControlSystems.jl

The following example simulates a feedback control system containing a PID controller using [ControlSystems.jl](https://juliacontrol.github.io/ControlSystems.jl) package. We simulate a response of the closed-loop system to the step disturbance $d(t) = 1$ entering at the *plant* (the system to be controlled) input, while the reference is $r(t) = 0$. 

```julia
using DiscretePIDs, ControlSystemsBase, Plots
Tf = 15                             # Simulation time
K  = 1                              # Proportional gain
Ti = 1                              # Integral time
Td = 1                              # Derivative time
Ts = 0.01                           # Sample time

P   = c2d(ss(tf(1, [1, 1])), Ts)    # Plant to be controlled, discretized using zero-order hold
pid = DiscretePID(; K, Ts, Ti, Td)

ctrl = function(x,t)
    y = (P.C*x)[]                   # Measured output
    d = 1                           # Disturbance
    r = 0                           # Reference
    u = pid(r, y)                   # Control
    u + d                           # Plant input is control + disturbance
end

res = lsim(P, ctrl, Tf)             # Simulate the closed-loop system

plot(res, plotu=true); ylabel!("u + d", sp=2)
```
![Simulation result](https://user-images.githubusercontent.com/3797491/172366365-c1533aed-e877-499d-9ebb-01df62107dfb.png)

Here we simulated a linear plant, in which case we were able to call `ControlSystems.lsim` specialized for linear systems. Below, we show two methods for simulation that works with a nonlinear plant, but we still use a linear system to make the comparison easier.

### Example using DifferentialEquations.jl

This example is identical to the one above except for using [DifferentialEquations.jl](https://docs.sciml.ai/DiffEqDocs/stable/) for the simulation, which makes it possible to consider more complex plants, in particular nonlinear ones.

There are several different ways one could go about including a discrete-time controller in a continuous-time simulation, in particular, we must choose a way to store the computed control variable. Two common approaches are

1. We use a global variable into which we write the control variable at each discrete time step.
2. We add an extra state variable to the system, and use it to store the control variable. 

In this example we choose the latter approach, since it has the added benefit of adding the computed control variable to the solution object.

We use `DiffEqCallbacks.PeriodicCallback`, with which we perform the PID-controller update, and store the computed control variable in the extra state variable.

```julia
using DiscretePIDs, ControlSystemsBase, OrdinaryDiffEq, DiffEqCallbacks, Plots

Tf = 15                             # Simulation time
K  = 1                              # Proportional gain
Ti = 1                              # Integral time
Td = 1                              # Derivative time
Ts = 0.01                           # Sample time

P = ss(tf(1, [1, 1]))               # Plant to be controlled in continuous time
A, B, C, D = ssdata(P)              # Extract the system matrices
pid = DiscretePID(; K, Ts, Ti, Td)

function dynamics!(dxu, xu, p, t)
    A, B, C, r, d = p               # Store the reference and disturbance in the parameter object
    x = xu[1:P.nx]                  # Extract the state
    u = xu[P.nx+1:end]              # Extract the control variable
    dxu[1:P.nx] .= A*x .+ B*(u .+ d)# Plant input is control variable + disturbance
    dxu[P.nx+1:end] .= 0            # Control variable has no dynamics, it's updated by the callback
end

cb = PeriodicCallback(Ts) do integrator
    p = integrator.p                # Extract the parameter object from the integrator
    (; C, r, d) = p                 # Extract the reference and disturbance from the parameter object
    x = integrator.u[1:P.nx]        # Extract the state (the integrator uses `u` to refer to the state, in control theory we typically name it `x`)
    y = (C*x)[]                     # Simulated measurement
    u = pid(r, y)                   # Compute the control variable
    integrator.u[P.nx+1:end] .= u   # Update the control-signal state variable 
end

parameters = (; A, B, C, r=0, d=1)  # Reference = 0, disturbance = 1
xu0 = zeros(P.nx + P.nu)            # Initial state of the system + control variables
prob = ODEProblem(dynamics!, xu0, (0, Tf), parameters, callback=cb)     # Reference = 0, disturbance = 1
sol = solve(prob, Tsit5(), saveat=Ts)

plot(sol, layout=(2, 1), ylabel=["x" "u"], lab="")
```

The figure should look more or less identical to the previous one, except that we plot the control variable $u$ instead of the combined input $u + d$ like we did above. Due to the fast sample rate $T_\mathrm{s}$, the control variable looks continuous, however, increase $T_s$ and you'll notice the zero-order-hold nature of $u$.

### Example using SeeToDee.jl

[SeeToDee.jl](https://baggepinnen.github.io/SeeToDee.jl/dev/) is a library of fixed-time-step integrators useful for "manual" (=one integration step at a time) simulation of control systems. The same example as above is simulated using [`SeeToDee.Rk4`](https://baggepinnen.github.io/SeeToDee.jl/dev/api/#SeeToDee.Rk4) here. The call to

```julia
discrete_dynamics = SeeToDee.Rk4(dynamics, Ts)
```
considers the continuous-time dynamical system modelled by
```math
\dot x(t) = f(x(t), u(t), p(t), t)
```
and at a given state $x$ and time $t$ and for a given control $u$, it computes an approximation $x^+$ to the state $x(t+T_\mathrm{s})$ at the next time step $t+T_\mathrm{s}$

```math
x(t+T_\mathrm{s}) \approx x^+ = \phi(x(t), u(t), p(t), t,T_\mathrm{s}).
```

```julia
using DiscretePIDs, ControlSystemsBase, SeeToDee, Plots

Tf = 15                             # Simulation time
K  = 1                              # Proportional gain
Ti = 1                              # Integral time
Td = 1                              # Derivative time
Ts = 0.01                           # Sample time
P  = ss(tf(1, [1, 1]))              # Plant to be controlled in continuous time
A,B,C = ssdata(P)                   # Extract the system matrices
p = (; A, B, C, r=0, d=1)           # Reference = 0, disturbance = 1

pid = DiscretePID(; K, Ts, Ti, Td)

ctrl = function(x,p,t)
    y = (p.C*x)[]                   # Measurement output of the plant
    pid(r, y)
end

function dynamics(x, u, p, t)       # This time we define the dynamics as a function of the state and the control
    A, B, C, r, d = p               # Store the reference and disturbance in the parameter object
    A*x .+ B*(u .+ d)               # Plant input is control variable + disturbance
end
discrete_dynamics = SeeToDee.Rk4(dynamics, Ts) # Create a discrete-time dynamics function

x = zeros(P.nx)                     # Initial condition
X, U = [], []                       # To store the solution 
t = range(0, step=Ts, stop=Tf)      # Time vector
for t = t
    u = ctrl(x, p, t)
    push!(U, u)                     # Save solution for plotting
    push!(X, x)
    x = discrete_dynamics(x, u, p, t) # Advance the state one step
end

Xm = reduce(hcat, X)'               # Reduce to from vector of vectors to matrix
Ym = Xm*P.C'                        # Compute the output (same as state in this simple case)
Um = reduce(hcat, U)'

plot(t, [Ym Um], layout=(2,1), ylabel = ["y" "u"], legend=false)
```
Once again, the output looks identical and is therefore omitted here.

## Details
- The derivative term only acts on the (filtered) measurement and not the command signal. It is thus safe to pass step changes in the reference to the controller. The parameter $b$ can further be set to zero to avoid step changes in the control variable in response to step changes in the reference.
- Bumpless transfer when updating `K` is realized by updating the state `I`. See the docs for `set_K!` for more details.
- The total control variable $u(t)$ (PID + feedforward) is limited by the integral anti-windup.
- The integrator is discretized using a forward difference (no direct term between the input and output through the integral state) while the derivative is discretized using a backward difference.
- This particular implementation of a discrete-time PID controller is detailed in Chapter 8 of [Wittenmark, Björn, Karl-Erik Årzén, and Karl Johan Åström. ‘Computer Control: An Overview’. IFAC Professional Brief. International Federation of Automatic Control, 2002](https://www.ifac-control.org/publications/list-of-professional-briefs/pb_wittenmark_etal_final.pdf/view).
- When used with input arguments of standard types, such as `Float64` or `Float32`, the controller is guaranteed not to allocate any memory or contain any dynamic dispatches. This analysis is carried out in the tests, and is performed using [AllocCheck.jl](https://github.com/JuliaLang/AllocCheck.jl).

### Simulation with fixed-point arithmetic
If the controller is ultimately to be implemented on a platform without floating-point hardware, we can simulate how it will behave with fixed-point arithmetics using the [FixedPointNumbers.jl](https://github.com/francescoalemanno/FixedPoint.jl) package. The following example modifies the first example above and shows how to simulate the controller using 16-bit fixed-point arithmetics with 10 bits for the fractional part:
```julia
using FixedPointNumbers
T = Fixed{Int16, 10}    # 16-bit fixed-point with 10 bits for the fractional part
pid = DiscretePID(; K = T(K), Ts = T(Ts), Ti = T(Ti), Td = T(Td))
res_fp = lsim(P, ctrl, Tf)
plot([res, res_fp], plotu=true, lab=["Float64" "" string(T) ""]); ylabel!("u + d", sp=2)
```
![Fixed-point simulation result](https://user-images.githubusercontent.com/3797491/249732319-0a3890d5-cb9c-45c2-93c7-20d3c7db0cf2.png)

The fixed-point controller behaves roughly the same in this case, but artifacts are clearly visible. If the number of bits used for the fractional part is decreased, the controller will start to misbehave.

## Compilation using JuliaC
> [!IMPORTANT]
>  At the time of writing, this requires a nightly version of julia. Consider this example to be highly experimental for now!

The file [`examples/juliac/juliac_pid.jl`](https://github.com/JuliaControl/DiscretePIDs.jl/blob/main/examples/juliac/juliac_pid.jl) contains a JuliaC-compatible interface that can be compiled into a C-callable shared library using JuliaC. To compile the file, run the following from the [`examples/juliac`](https://github.com/JuliaControl/DiscretePIDs.jl/tree/main/examples/juliac) folder:
```bash
julia +nightly --project <PATH_TO_JULIA_REPO>/julia/contrib/juliac.jl --output-lib juliac_pid --experimental --trim=unsafe-warn --compile-ccallable juliac_pid.jl
```
where `<PATH_TO_JULIA_REPO>` should be replaced with the path to the Julia repository on your system. The command will generate a shared library `juliac_pid` that can be called from C. The file [`examples/juliac/juliac_pid.h`](https://github.com/JuliaControl/DiscretePIDs.jl/blob/main/examples/juliac/juliac_pid.h) contains the C-compatible interface to the shared library. The C program may be compiled with a command like
```bash
export LD_LIBRARY_PATH=<PATH_TO_JULIA_REPO>/julia/usr/lib:$LD_LIBRARY_PATH
gcc -o pid_program test_juliac_pid.c -I <PATH_TO_JULIA_REPO>/julia/usr/include/julia -L<PATH_TO_JULIA_REPO>/julia/usr/lib -ljulia -ldl
```
and then run by
```bash
./pid_program
```
which should produce the output
```
DiscretePIDs/examples/juliac> ./pid_program 
Loading juliac_pid.so
Loaded juliac_pid.so
Finding symbols
Found all symbols!
calculate_control! returned: 1.000000
calculate_control! returned: 2.000000
calculate_control! returned: 3.000000
calculate_control! returned: 3.000000
calculate_control! returned: 3.000000
```

## See also
- [TrajectoryLimiters.jl](https://github.com/baggepinnen/TrajectoryLimiters.jl) To generate dynamically feasible reference trajectories with bounded velocity and acceleration given an instantaneous reference $r(t)$ which may change abruptly.
- [SymbolicControlSystems.jl](https://github.com/JuliaControl/SymbolicControlSystems.jl) For C-code generation of LTI systems.