# NOTE: it is currently not possible to call a julia-produced shared-library from julia.
# To test the compiled shared library, see test_juliac_pid.c instead.
cd(@__DIR__)

const T = Float64
@info("Loading juliac_pid.so")
lib = Libc.Libdl.dlopen("/home/fredrikb/.julia/dev/DiscretePIDs/examples/juliac/juliac_pid.so")
@info("Loaded juliac_pid.so, finding calculate_control!")
const calc = Libc.Libdl.dlsym(lib, :calculate_control!)
@info("Found calculate_control!")

function pid(r::T, y::T, uff::T)
    ccall(calc, T, (T, T, T), r, y, uff)
end

pid(0.0, 0.0, 0.0) # test

using ControlSystemsBase, Plots
Tf = 15   # Simulation time
Ts = 0.01 # sample time

P   = c2d(ss(tf(1, [1, 1])), Ts) # Process to be controlled, discretized using zero-order hold

ctrl = function(x,t)
    y = (P.C*x)[] # measurement
    d = 1         # disturbance
    r = 0         # reference
    u = pid(T(r), T(y), T(0))
    u + d # Plant input is control signal + disturbance
end

res = lsim(P, ctrl, Tf)

plot(res, plotu=true); ylabel!("u + d", sp=2)