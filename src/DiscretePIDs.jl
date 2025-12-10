module DiscretePIDs

export DiscretePID, calculate_control!, set_K!, set_Td!, set_Ti!, reset_state!, parallel2standard

using Printf

"""
    DiscretePID{T}
"""
mutable struct DiscretePID{T} <: Function
    "Proportional gain"
    K::T 
    "Integral time"
    Ti::T 
    "Derivative time"
    Td::T 
    "Reset time"
    Tt::T 
    "Maximum derivative gain"
    const N::T 
    "Fraction of set point in prop. term"
    wp::T
    "Fraction of set point in derivative term"
    wd::T
    "Low output limit"
    umin::T 
    "High output limit"
    umax::T 
    "Sampling period"
    const Ts::T 
    bi::T
    const ar::T
    bd::T
    ad::T
    "Integral state"
    I::T 
    "Derivative state"
    D::T 
    "Last derivative error (wd*r - y)"
    yold::T 
end

"""
    DiscretePID(; K = 1, Ti = false, Td = false, Tt = √(Ti*Td), N = 10, wp = 1, wd = 0, umin = -Inf, umax = Inf, Ts, I = 0, D = 0, yold = 0)

A discrete-time PID controller with set-point weighting and integrator anti-windup.
The controller is implemented on the standard form
```math
u = K \\left( e + \\dfrac{1}{Ti} \\int e dt + T_d \\dfrac{de}{dt} \\right)
```

```math
U(s) = K \\left( wp R(s) - Y(s) + \\dfrac{1}{sT_i} \\left( R(s) Y(s) \\right) - \\dfrac{sT_d}{1 + s T_d / N}(Y(s) - w_d R(s))
```

Call the controller like this
```julia
u = pid(r, y, uff) # uff is optional
u = calculate_control!(pid, r, y, uff) # Equivalent to the above
```

# Arguments:
- `K`: Proportional gain
- `Ti`: Integral time
- `Td`: Derivative time
- `Tt`: Reset time for anti-windup
- `N`: Maximum derivative gain
- `wp`: Fraction of set point in proportional term
- `wd`: Fraction of set point in derivative term (default 0)
- `umin`: Low output limit
- `umax`: High output limit
- `Ts`: Sampling period
- `I`: Integral part
- `D`: Derivative part
- `yold`: Last derivative error (wd*r - y)

See also [`calculate_control!`](@ref), [`set_K!`](@ref), [`set_Ti!`](@ref), [`set_Td!`](@ref), [`reset_state!`](@ref).
"""
function DiscretePID(;
    K::T  = 1f0,
    Ti = false,
    Td = false,
    Tt = Ti > 0 && Td > 0 ? typeof(K)(√(Ti*Td)) : typeof(K)(10),
    N  = typeof(K)(10),
    wp  = typeof(K)(1),
    wd = zero(typeof(K)),
    b = nothing,
    umin = typemin(typeof(K)),
    umax = typemax(typeof(K)),
    Ts,
    I    = zero(typeof(K)),
    D    = zero(typeof(K)),
    yold = zero(typeof(K)),
) where T
    if Ti > 0
        bi = K * Ts / Ti
    else
        bi = zero(K * Ts)
    end
    if b !== nothing
        wp = typeof(K)(b)
        @warn "Parameter `b` is deprecated. Use `wp` instead."
    end
    Tt ≥ 0 || throw(ArgumentError("Tt must be positive"))
    Td ≥ 0 || throw(ArgumentError("Td must be positive"))
    N ≥ 0 || throw(ArgumentError("N must be positive"))
    0 ≤ wp ≤ 1 || throw(ArgumentError("wp must be ∈ [0, 1]"))
    0 ≤ wd ≤ 1 || throw(ArgumentError("wd must be ∈ [0, 1]"))
    umax > umin || throw(ArgumentError("umax must be greater than umin"))

    if Ti > 0
        ar = Ts / Tt
    else
        ar = zero(Ts / Tt)
    end
    ad = Td / (Td + N * Ts)
    bd = K * N * ad

    T2 = promote_type(typeof.((K, Ti, Td, Tt, N, wp, wd, umin, umax, Ts, bi, ar, bd, ad, I, D, yold))...)

    DiscretePID(T2.((K, Ti, Td, Tt, N, wp, wd, umin, umax, Ts, bi, ar, bd, ad, I, D, yold))...)
end

"""
    set_K!(pid::DiscretePID, K, r, y)

Update `K` in the PID controller. This function takes the current reference and measurement as well in order to provide bumpless transfer. This is realized by updating the internal state `I`.

Note: Due to the bumpless transfer, setting ``K = 0`` does not imply that the controller output will be 0 if the integral state is non zero. To reset the controller state, call `reset_state!(pid)`.
"""
function set_K!(pid::DiscretePID, K, r, y)
    Kold = pid.K
    pid.K = K
    pid.bd = K * pid.N * pid.ad
    if pid.Ti > 0
        pid.bi = K * pid.Ts / pid.Ti
        pid.I = pid.I + Kold*(pid.wp*r - y) - K*(pid.wp*r - y)
    end
    nothing
end

"""
    set_Ti!(pid::DiscretePID, Ti)

Update `Ti` in the PID controller.
"""
function set_Ti!(pid::DiscretePID{T}, Ti) where T
    Ti < 0 && throw(ArgumentError("Thou shall not use negative Ti values."))
    pid.Ti = Ti
    if Ti > 0
        pid.bi = pid.K * pid.Ts / Ti
    else
        pid.bi = zero(T)
    end
    nothing
end

"""
    set_Td!(pid::DiscretePID, Td)

Update `Td` in the PID controller.
"""
function set_Td!(pid::DiscretePID, Td)
    Td < 0 && throw(ArgumentError("Thou shall not use negative Td values."))
    pid.Td = Td
    pid.ad = Td / (Td + pid.N * pid.Ts)
    pid.bd = pid.K * pid.N * pid.ad
    nothing
end


"""
    u = calculate_control!(pid::DiscretePID, r, y, uff=0; yd = nothing)
    (pid)(r, y, uff=0) # Alternative syntax

Calculate the control output from the PID controller when `r` is the reference (set point), `y` is the latest measurement and `uff` is the feed-forward contribution.
If the type of the input arguments differ from the numeric type used by the PID controller, they will be converted before computations are performed.

The derivative term is by default computed by filtering the measurement `y`, but it can also be provided directly by setting the optional keyword argument `yd`. When `yd` is set, no filtering is applied by the PID controller, i.e., `N` is ignored.
"""
function calculate_control!(pid::DiscretePID{T}, r0, y0, uff0=0; yd=nothing) where T
    r = T(r0)
    y = T(y0)
    uff = T(uff0)
    P = pid.K * (pid.wp * r - y)
    e = pid.wd * r - y  # weighted error for derivative
    if yd === nothing
        pid.D = pid.ad * pid.D + pid.bd * (e - pid.yold)
    else
        pid.D = - pid.K * pid.Td * T(yd)
    end
    v = P + pid.I + pid.D + uff
    u = clamp(v, pid.umin, pid.umax)
    pid.I = pid.I + pid.bi * (r - y) + pid.ar * (u - v)
    pid.yold = e  # store weighted error for next derivative calculation
    return u
end

(pid::DiscretePID)(args...) = calculate_control!(pid, args...)

function Base.show(io::IO, ::MIME"text/plain", pid::DiscretePID)
    println(io, "$(typeof(pid))( # with parameters and state:")
    for name in fieldnames(DiscretePID)
        @printf(io, "    %-14.7g,# %s\n", getfield(pid, name), name)
    end
    println(io, ")")
end

"""
    reset_state!(pid::DiscretePID)

Set all internal state variables to zero (`I`, `D` and `yold`).
"""
function reset_state!(pid::DiscretePID)
    pid.I = zero(pid.I)
    pid.D = zero(pid.D)
    pid.yold = zero(pid.yold)
    nothing
end

"""
    K, Ti, Td = parallel2standard(Kp, Ki, Kd)

Convert parameters from form "parallel" form
``K_p + K_i/s + K_d s``

to "standard" form used in DiscretePID:
``K(1 + 1/(T_i s) + T_d s)``

You may provide either three arguments or an array with three elements in the same order.
"""
function parallel2standard(Kp, Ki, Kd)
    Kp == 0 && throw(DomainError("Cannot convert to standard form when Kp=0"))
    return (Kp, Kp / Ki, Kd / Kp)
end

"""
    K, Ti, Td, N = parallel2standard(Kp, Ki, Kd, Tf)

Convert parameters from form "parallel" form with first-order filter
``K_p (w_p r-y) + K_i (r-y)/s - K_d s y/(Tf s + 1)``

to "standard" form used in DiscretePID:
``K (w_p r-y + (r-y)/(T_i s) - T_d s y/(T_d / N s + 1))``

You may provide either four arguments or an array with four elements in the same order.
"""
function parallel2standard(Kp, Ki, Kd, Tf)
    Kp, Ti, Td = parallel2standard(Kp, Ki, Kd)
    N = Td / Tf
    return (Kp, Ti, Td, N)
end

function parallel2standard(p)
    return [parallel2standard(p...)...]
end

end
