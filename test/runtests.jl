using DiscretePIDs
using Test
using ControlSystems

@testset "DiscretePIDs.jl" begin
    

K = 1
Ts = 0.01

pid = DiscretePID(; K, Ts)
display(pid)

ctrl = function(x,t)
    y = (P.C*x)[]
    r = 1
    u = pid(r, y)
end

P = c2d(ss(tf(1, [1, 1])), Ts)

## P control
C = c2d(ControlSystems.pid(kp = K), Ts)
res = step(feedback(P*C), 3)
res2 = lsim(P, ctrl, 3)

# plot([res, res2])
@test res.y ≈ res2.y


## PI control
Ti = 1
pid = DiscretePID(; K, Ts, Ti)
C = c2d(ControlSystems.pid(kp = K, ki = Ti, series=true, time=true), Ts)
res = step(feedback(P*C), 3)
res2 = lsim(P, ctrl, 3)

# plot([res, res2])
@test res.y ≈ res2.y rtol=0.01

## PID control
# Here we simulate a load disturbance instead since the discrete PID does not differentiate r, while the ControlSystems.pid does.
Tf = 10
Ti = 1
Td = 1
pid = DiscretePID(; K, Ts, Ti, Td)

ctrl = function(x,t)
    y = (P.C*x)[]
    d = 1
    r = 0
    u = pid(r, y)
    u + d
end

C = let s = tf("s")
    ki = 1/Ti
    kd = Td
    C0 = K*(one(K) + one(K)/(ki*s) + kd*s/(1 + kd/pid.N*s))
    c2d(C0, Ts)
end


res = step(feedback(P, C), Tf)
res2 = lsim(P, ctrl, Tf)

@test res.y ≈ res2.y rtol=0.02
# plot([res, res2])


## PI control with sp weighting
Tf = 10
Ti = 1
b = 0.0
pid = DiscretePID(; K, Ts, Ti, b)
ctrl = function(x,t)
    y = (P.C*x)[]
    r = 1
    u = pid(r, y)
end

C = c2d(ControlSystems.pid(kp = K, ki = Ti, series=true, time=true), Ts)
res = step(feedback(P*C), Tf)
res2 = lsim(P, ctrl, Tf)

# plot([res, res2])
# @test res.y ≈ res2.y rtol=0.01


## PID control with bumpless transfer
# Here we simulate a load disturbance instead since the discrete PID does not differentiate r, while the ControlSystems.pid does.
Tf = 10
Ti = 1
Td = 1
pid = DiscretePID(; K, Ts, Ti, Td)

ctrl = function(x,t)
    y = (P.C*x)[]
    d = 1
    r = 0
    t == 3 && set_K!(pid, 2, r, y)
    t == 4 && set_Ti!(pid, 2)
    t == 5 && set_Td!(pid, 2)
    u = pid(r, y)
    u + d
end

C = let s = tf("s")
    ki = 1/Ti
    kd = Td
    C0 = K*(one(K) + one(K)/(ki*s) + kd*s/(1 + kd/pid.N*s))
    c2d(C0, Ts)
end


res = step(feedback(P, C), Tf)
res2 = lsim(P, ctrl, Tf)

@test maximum(diff(res2.u[:])) < 0.01
# plot([res, res2], plotu=true)


## P control with saturation
umax = 0.7
pid = DiscretePID(; K, Ts, umax)
ctrl = function(x,t)
    y = (P.C*x)[]
    r = 1
    u = pid(r, y)
end
C = c2d(ControlSystems.pid(kp = K), Ts)
res = step(feedback(P*C), 3)
res2 = lsim(P, ctrl, 3)

# plot([res, res2], plotu=true)
@test maximum(res2.u) == umax

end