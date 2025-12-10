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

## PID control with external derivative (yd keyword)
# Compare internal filtered derivative vs externally provided derivative
pid_internal = DiscretePID(; K = 1.0*K, Ts, Ti, Td = 0.8*Td)
pid_external = DiscretePID(; K = 1.0*K, Ts, Ti, Td = 0.8*Td)

global yold_ext = 0.0
ctrl_internal = function(x, t)
    y = (P.C*x)[]
    d = 1
    r = 0
    u = pid_internal(r, y)
    u + d
end

ctrl_external = function(x, t)
    global yold_ext
    y = (P.C*x)[]
    d = 1
    r = 0
    yd = (y - yold_ext) / Ts  # Compute raw derivative of -y
    yold_ext = y
    u = calculate_control!(pid_external, r, y; yd)
    u + d
end

res_internal = lsim(P, ctrl_internal, Tf)
reset_state!(pid_internal)
reset_state!(pid_external)
yold_ext = 0.0
res_external = lsim(P, ctrl_external, Tf)

# External derivative is unfiltered, so allow larger tolerance
@test res_internal.y ≈ res_external.y rtol=0.1

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


## PID control with derivative set-point weighting (wd parameter)
# Compare wd=0 (derivative only on -y) vs wd=1 (derivative on r-y)
Tf = 30
Ti = 1
Td = 1
pid_wd0 = DiscretePID(; K, Ts, Ti, Td, wd=0)
pid_wd1 = DiscretePID(; K, Ts, Ti, Td, wd=1)

ctrl_wd0 = function(x, t)
    y = (P.C*x)[]
    r = (t >= 5)  # Step in reference
    pid_wd0(r, y)
end

ctrl_wd1 = function(x, t)
    y = (P.C*x)[]
    r = (t >= 5)  # Step in reference
    pid_wd1(r, y)
end

res_wd0 = lsim(P, ctrl_wd0, Tf)
res_wd1 = lsim(P, ctrl_wd1, Tf)

# With wd=1, step in r causes derivative kick; with wd=0 it doesn't
# So the control signals should differ, especially around t=5
@test res_wd0.y != res_wd1.y  # Results should be different
@test maximum(abs.(res_wd1.u)) > 5*maximum(abs.(res_wd0.u))  # wd=1 has larger control signal due to derivative kick


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

kpkikdTf = rand(4)
kp, ki, kd, Tf = kpkikdTf
ps = parallel2standard(kpkikdTf)
K,Ti,Td,N = ps

@test ControlSystemsBase.pid(kp, ki, kd; Tf, form = :parallel, filter_order=1) ≈ ControlSystemsBase.pid(K, Ti, Td; Tf=Td/N, form = :standard, filter_order=1)


end
