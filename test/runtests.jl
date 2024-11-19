using DiscretePIDs
using Test
using ControlSystemsBase
using AllocCheck
using JET

@testset "DiscretePIDs.jl" begin

# Allocation tests
@testset "allocation" begin
    @info "Testing allocation"
    for T0 = (Float64, Float32)
        for T1 = (Float64, Float32, Int)
            for T2 = (Float64, Float32, Int)
                for T3 = (Float64, Float32, Int, Nothing)
                    @test isempty(AllocCheck.check_allocs(calculate_control!, (DiscretePID{T0}, T1, T2)))
                    @test isempty(AllocCheck.check_allocs(calculate_control!, (DiscretePID{T0}, T1, T2, T3)))
                end
            end
        end
    end
end
    

K = 1
Ts = 0.01

pid = DiscretePID(; K, Ts)
display(pid)
for T1 = (Float64, Float32, Int)
    for T2 = (Float64, Float32, Int)
        for T3 = (Float64, Float32, Int, Nothing)
            @test isempty(AllocCheck.check_allocs(pid, (T1, T2)))
            @test isempty(AllocCheck.check_allocs(pid, (T1, T2, T3)))
        end
    end
end

ctrl = function(x,t)
    y = (P.C*x)[]
    r = 1
    u = pid(r, y)
end

P = c2d(ss(tf(1, [1, 1])), Ts)

## P control
C = c2d(ControlSystemsBase.pid(K, 0), Ts)
res = step(feedback(P*C), 3)
res2 = lsim(P, ctrl, 3)

# plot([res, res2])
@test res.y ≈ res2.y


## PI control
Ti = 1
pid = DiscretePID(; K, Ts, Ti)
C = c2d(ControlSystemsBase.pid(K, Ti), Ts)
res = step(feedback(P*C), 3)
res2 = lsim(P, ctrl, 3)

# plot([res, res2])
@test res.y ≈ res2.y rtol=0.01

## PID control
# Here we simulate a load disturbance instead since the discrete PID does not differentiate r, while the ControlSystemsBase.pid does.
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

reset_state!(pid)
res3 = lsim(P, ctrl, Tf)
@test res3.y == res2.y

@test_opt pid(1.0, 1.0)
@test_opt pid(1.0, 1.0, 1.0)
# @report_call pid(1.0, 1.0)

## Test with FixedPointNumbers
using FixedPointNumbers
T = Fixed{Int16, 10} # 16-bit signed fixed-point with 10 bits for the fractional part
pid = DiscretePID(; K = T(K), Ts = T(Ts), Ti = T(Ti), Td = T(Td))
@test pid isa DiscretePID{T}

res3 = lsim(P, ctrl, Tf)

@test res.y ≈ res3.y rtol=0.05


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

C = c2d(ControlSystemsBase.pid(K, Ti), Ts)
res = step(feedback(P*C), Tf)
res2 = lsim(P, ctrl, Tf)

# plot([res, res2])
# @test res.y ≈ res2.y rtol=0.01


## PID control with bumpless transfer
# Here we simulate a load disturbance instead since the discrete PID does not differentiate r, while the ControlSystemsBase.pid does.
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
C = c2d(ControlSystemsBase.pid(K, 0), Ts)
res = step(feedback(P*C), 3)
res2 = lsim(P, ctrl, 3)

# plot([res, res2], plotu=true)
@test maximum(res2.u) == umax
@test pid.I == 0.0


@test DiscretePID(Ts=1f0) isa DiscretePID{Float32}
@test DiscretePID(Ts=1.0) isa DiscretePID{Float64}

end
