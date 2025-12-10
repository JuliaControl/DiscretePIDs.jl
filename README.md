# DiscretePIDs

[![Build Status](https://github.com/JuliaControl/DiscretePIDs.jl/actions/workflows/CI.yml/badge.svg?branch=main)](https://github.com/JuliaControl/DiscretePIDs.jl/actions/workflows/CI.yml?query=branch%3Amain)
[![Coverage](https://codecov.io/gh/JuliaControl/DiscretePIDs.jl/branch/main/graph/badge.svg)](https://codecov.io/gh/JuliaControl/DiscretePIDs.jl)


This package implements a discrete-time PID controller as an approximation of the continuous-time PID controller given by
$$U(s) = K \left( w_p R(s) - Y(s) + \dfrac{1}{sT_i} \left( R(s) - Y(s) \right) + \dfrac{sT_d}{1 + s T_d / N}(w_d R(s) - Y(s)) \right) + U_\textrm{ff}(s),$$
where
- $u(t) \leftrightarrow U(s)$ is the control signal
- $y(t) \leftrightarrow Y(s)$ is the measurement signal
- $r(t) \leftrightarrow R(s)$ is the reference / set point
- $u_\textrm{ff}(t) \leftrightarrow U_\textrm{ff}(s)$ is the feed-forward contribution
- $K$ is the proportional gain
- $T_i$ is the integral time
- $T_d$ is the derivative time
- $N$ is a parameter that limits the gain of the derivative term at high frequencies, typically ranges from 2 to 20,
- $w_p \in [0, 1]$ is a parameter that gives the proportion of the reference signal that appears in the proportional term.
- $w_d \in [0, 1]$ is a parameter that gives the proportion of the reference signal that appears in the derivative term (default 0).

*Saturation* of the controller output is parameterized by $u_{\min}$ and $u_{\max}$, and the integrator *anti-windup* is parameterized by the tracking time $T_\mathrm{t}$.

## Usage

Construct a controller by
```julia
pid = DiscretePID(; K = 1, Ti = false, Td = false, Tt = √(Ti*Td), N = 10, wp = 1, wd = 0, umin = -Inf, umax = Inf, Ts, I = 0, D = 0, yold = 0)
```
and compute the control signal at a given time using
```julia
u = pid(r, y, uff)
```
or
```julia
u = calculate_control!(pid, r, y, uff)
```

The derivative term is by default computed by filtering the measurement $y$, but it can also be sourced externally by setting the keyword argument `yd`:
```julia
u = calculate_control!(pid, r, y, uff; yd)
```
When `yd` is provided, no filtering is applied by the PID controller, i.e., $N$ is ignored. This is useful when the derivative is computed externally, e.g., from a velocity sensor or an observer.

The parameters $K$, $T_i$, and $T_d$ may be updated using the functions `set_K!`, `set_Ti!`, and `set_Td!`, respectively.

The numeric type used by the controller (the `T` in `DiscretePID{T}`) is determined by the types of the parameters. To use a custom number type, e.g., a fixed-point number type, simply pass the parameters as that type, see example below. The controller will automatically convert measurements and references to this type before performing the control calculations.

The **internal state** of the controller can be reset to zero using the function `reset_state!(pid)`. If repeated simulations using the same controller object are performed, the state should likely be reset between simulations.

## Examples

### Example using ControlSystems.jl

The following example simulates a feedback control system containing a PID controller using [ControlSystems.jl](https://juliacontrol.github.io/ControlSystems.jl) package. We simulate a response of the closed-loop system to the step disturbance $d(t) = 1$ entering at the *plant* (the system to be controlled) input, while the reference is $r(t) = 0$. 

```julia
using DiscretePIDs, ControlSystemsBase, Plots
Tf = 30   # Simulation time
K  = 1    # Proportional gain
Ti = 1    # Integral time
Td = 1    # Derivative time
Ts = 0.01 # sample time

P   = c2d(ss(tf(1, [1, 1])), Ts) # Process to be controlled, discretized using zero-order hold
pid = DiscretePID(; K, Ts, Ti, Td)

ctrl = function(x,t)
    y = (P.C*x)[] # measurement
    d = 1         # disturbance
    r = (t >= 15) # reference
    u = pid(r, y) # control signal
    u + d # Plant input is control signal + disturbance
end

res = lsim(P, ctrl, Tf)

plot(res, plotu=true); ylabel!("u + d", sp=2)
```
![Simulation result](https://github.com/user-attachments/assets/2de34be7-4811-4801-b6ca-bc4c932b3331)

Here we simulated a linear plant, in which case we were able to call `ControlSystems.lsim` specialized for linear systems. Below, we show two methods for simulation that works with a nonlinear plant, but we still use a linear system to make the comparison easier.

For comparison, we also perform the same simulation with a two degree-of-freedom PID controller
```julia
using ControlSystemsBase, DiscretePIDs, Plots
t = 0:Ts:Tf
u = [ones(length(t)) t .>= 15]' # Input signal [d; r]
C = pid_2dof(K, Ti, Td; Ts, N=10)
Gcl = feedback(P, C, W1=1, U2=2, W2=1, Z2=1, pos_feedback=true)
simres = lsim(Gcl, u)
plot(simres, plotu=true, lab=["y" "u" "d" "r"], layout=(2,1), sp=[1 2 2 1], ylabel="")
```
![Simulation result](https://github.com/user-attachments/assets/1267fc64-72f1-4560-ba66-ccf88bcae150)


Please note: The result of simulation with a controller computed by `pid_2dof` will be _slightly_ different due to a difference in discretization. DiscretePIDs uses a forward-Euler approximation for the integrator and a backward Euler approximation for the derivative, while `pid_2dof` uses a Tustin approximation (default) for both.

### Example using DifferentialEquations.jl

This example is identical to the one above except for using [DifferentialEquations.jl](https://docs.sciml.ai/DiffEqDocs/stable/) for the simulation, which makes it possible to consider more complex plants, in particular nonlinear ones.

There are several different ways one could go about including a discrete-time controller in a continuous-time simulation, in particular, we must choose a way to store the computed control variable. Two common approaches are

1. We use a global variable into which we write the control signal at each discrete time step.
2. We add an extra state variable to the system, and use it to store the control variable. 

In this example we choose the latter approach, since it has the added benefit of adding the computed control variable to the solution object.

We use `DiffEqCallbacks.PeriodicCallback`, in which we perform the PID-controller update, and store the computed control signal in the extra state variable.

```julia
using DiscretePIDs, ControlSystemsBase, OrdinaryDiffEq, DiffEqCallbacks, Plots

Tf = 30   # Simulation time
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
    (; C, d) = p     # Extract the reference and disturbance from the parameter object
    x = integrator.u[1:P.nx] # Extract the state (the integrator uses the variable name `u` to refer to the state, in control theory we typically use the variable name `x`)
    r = (integrator.t >= 15) # Reference
    y = (C*x)[]         # Simulated measurement
    u = pid(r, y)       # Compute the control signal
    integrator.u[P.nx+1:end] .= u # Update the control-signal state variable 
end

parameters = (; A, B, C, d=1) # disturbance = 1
xu0 = zeros(P.nx + P.nu) # Initial state of the system + control signals
prob = ODEProblem(dynamics!, xu0, (0, Tf), parameters, callback=cb) # disturbance = 1
sol = solve(prob, Tsit5(), saveat=Ts)

plot(sol, layout=(2, 1), ylabel=["x" "u"], lab="")
```
The figure should look more or less identical to the one above, except that we plot the control signal $u$ instead of the combined input $u + d$ like we did above. Due to the fast sample rate $T_s$, the control signal looks continuous, however, increase $T_s$ and you'll notice the zero-order-hold nature of $u$.

### Example using SeeToDee.jl

[SeeToDee.jl](https://baggepinnen.github.io/SeeToDee.jl/dev/) is a library of fixed-time-step integrators useful for "manual" (=one integration step at a time) simulation of control systems. The same example as above is simulated using [`SeeToDee.Rk4`](https://baggepinnen.github.io/SeeToDee.jl/dev/api/#SeeToDee.Rk4) here. The call to

```julia
discrete_dynamics = SeeToDee.Rk4(dynamics, Ts)
```
considers the continuous-time dynamical system modelled by
```math
\dot x(t) = f(x(t), u(t), p(t), t)
```
and at a given state $x$ and time $t$ and for a given control $u$, it computes an approximation $x^+$ to the state $x(t+T_s)$ at the next time step $t+T_s$

```math
x(t+T_s) \approx x^+ = \phi(x(t), u(t), p(t), t,T_s).
```

```julia
using DiscretePIDs, ControlSystemsBase, SeeToDee, Plots
Tf = 30   # Simulation time
K  = 1    # Proportional gain
Ti = 1    # Integral time
Td = 1    # Derivative time
Ts = 0.01 # sample time
P  = ss(tf(1, [1, 1]))    # Process to be controlled, in continuous time
A,B,C = ssdata(P)         # Extract the system matrices
p = (; A, B, C, d=1) # reference = 0, disturbance = 1

pid = DiscretePID(; K, Ts, Ti, Td)

ctrl = function(x,p,t)
    r = (t >= 15)   # reference
    y = (p.C*x)[]   # measurement
    pid(r, y)
end

function dynamics(x, u, p, t) # This time we define the dynamics as a function of the state and control signal
    A, B, C, d = p   # We store the reference and disturbance in the parameter object
    A*x .+ B*(u .+ d) # Plant input is control signal + disturbance
end
discrete_dynamics = SeeToDee.Rk4(dynamics, Ts) # Create a discrete-time dynamics function

x = zeros(P.nx) # Initial condition
X, U = [], []   # To store the solution 
t = range(0, step=Ts, stop=Tf) # Time vector
for t = t
    u = ctrl(x, p, t)
    push!(U, u) # Save solution for plotting
    push!(X, x)
    x = discrete_dynamics(x, u, p, t) # Advance the state one step
end

Xm = reduce(hcat, X)' # Reduce to from vector of vectors to matrix
Ym = Xm*P.C'          # Compute the output (same as state in this simple case)
Um = reduce(hcat, U)'

plot(t, [Ym Um], layout=(2,1), ylabel = ["y" "u"], legend=false)
```
Once again, the output looks identical and is therefore omitted here.

## Parameter conversion
The form of the PID controller used in this package is often referred to as ["standard form"](https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Standard_form). If you have PID parameters on "parallel form"
```math
K_p (br-y) + K_i (r-y)/s - K_d s y/(Tf s + 1)
```
you may convert these to the standard form
```math
K (b r - y + 1/T_i (r - y) - s T_d y/(1 + s T_d / N))
```
using the function `K, Ti, Td = parallel2standard(kp, ki, kd)` or, if a filter parameter is included, `K, Ti, Td, N = parallel2standard(kp, ki, kd, Tf)`. This function also accepts a vector of parameters in the same order, in which case a vector is returned.

## Details
- The derivative term by default only acts on the (filtered) measurement and not the command signal. It is thus safe to pass step changes in the reference to the controller. Set `wd = 1` to let the derivative act on the error `r-y` instead. The parameter $w_p$ can further be set to zero to avoid step changes in the control signal in response to step changes in the reference.
- Bumpless transfer when updating `K` is realized by updating the state `I`. See the docs for `set_K!` for more details.
- The total control signal $u(t)$ (PID + feedforward) is limited by the integral anti-windup.
- The integrator is discretized using a forward difference (no direct term between the input and output through the integral state) while the derivative is discretized using a backward difference. This approximation has the advantage that it is always stable and that the sampled pole goes to zero when $T_d$ goes to zero. Tustin's approximation gives an approximation such that the pole instead goes to $z = −1$ as $T_d$ goes to zero. 
- This particular implementation of a discrete-time PID controller is detailed in Chapter 8 of [Wittenmark, Björn, Karl-Erik Årzén, and Karl Johan Åström. ‘Computer Control: An Overview’. IFAC Professional Brief. International Federation of Automatic Control, 2002](https://www.ifac-control.org/publications/list-of-professional-briefs/pb_wittenmark_etal_final.pdf/view).
- When used with input arguments of standard types, such as `Float64` or `Float32`, the controller is guaranteed not to allocate any memory or contain any dynamic dispatches. This analysis is carried out in the tests, and is performed using [AllocCheck.jl](https://github.com/JuliaLang/AllocCheck.jl).

### Simulation with fixed-point arithmetic
If the controller is ultimately to be implemented on a platform without floating-point hardware, we can simulate how it will behave with fixed-point arithmetics using the [FixedPointNumbers.jl](https://github.com/francescoalemanno/FixedPoint.jl) package. The following example modifies the first example above and shows how to simulate the controller using 16-bit fixed-point arithmetics with 10 bits for the fractional part:
```julia
using FixedPointNumbers
T = Fixed{Int16, 10} # 16-bit fixed-point with 10 bits for the fractional part
pid = DiscretePID(; K = T(K), Ts = T(Ts), Ti = T(Ti), Td = T(Td))
res_fp = lsim(P, ctrl, Tf)
plot([res, res_fp], plotu=true, lab=["Float64" "" string(T) ""]); ylabel!("u + d", sp=2)
```
![Fixed-point simulation result](https://user-images.githubusercontent.com/3797491/249732319-0a3890d5-cb9c-45c2-93c7-20d3c7db0cf2.png)

The fixed-point controller behaves roughly the same in this case, but artifacts are clearly visible. If the number of bits used for the fractional part is decreased, the controller will start to misbehave.

## Compilation using JuliaC

This demonstration is part of the examples in the article:
> [Bagge Carlson, et al. "C-code generation considered unnecessary: go directly to binary, do not pass C. Compilation of Julia code for deployment in model-based engineering." arXiv preprint arXiv:2502.01128 (2025).](https://arxiv.org/abs/2502.01128)

The file [`examples/juliac/juliac_pid.jl`](https://github.com/JuliaControl/DiscretePIDs.jl/blob/main/examples/juliac/juliac_pid.jl) contains a JuliaC-compatible interface that can be compiled into a C-callable shared library using JuliaC.
First, install JuliaC by running `pkg> app add JuliaC` in the Julia REPL. Then, to compile the file, run the following from the [`examples/juliac`](https://github.com/JuliaControl/DiscretePIDs.jl/tree/main/examples/juliac) folder:
```bash
juliac --output-lib juliac_pid --experimental --trim=unsafe-warn --compile-ccallable --project=. juliac_pid.jl
```
The command will generate a shared library `juliac_pid.so` (or `.dylib`/`.dll`) that can be called from C.
The file [`examples/juliac/juliac_pid.h`](https://github.com/JuliaControl/DiscretePIDs.jl/blob/main/examples/juliac/juliac_pid.h) contains the C-compatible interface to the shared library.
The C program may be compiled with a command like
```bash
gcc -o pid_program test_juliac_pid.c -I $HOME/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/include/julia -L$HOME/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/lib -ljulia -ldl
```
(modify the Julia path to match your installation)
and then run by
```bash
export LD_LIBRARY_PATH=$HOME/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/lib:$HOME/.julia/juliaup/julia-1.12.1+0.x64.linux.gnu/lib/julia:$LD_LIBRARY_PATH
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