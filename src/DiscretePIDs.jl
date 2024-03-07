module DiscretePIDs

export DiscretePID, calculate_control!, set_K!, set_Td!, set_Ti!

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
    N::T 
    "Fraction of set point in prop. term"
    b::T 
    "Low output limit"
    umin::T 
    "High output limit"
    umax::T 
    "Sampling period"
    Ts::T 
    bi::T
    ar::T
    bd::T
    ad::T
    "Integral state"
    I::T 
    "Derivative state"
    D::T 
    "Last measurement signal"
    yold::T 
end

floattype(K) = float(typeof(K))

"""
    DiscretePID(; K = 1, Ti = false, Td = false, Tt = √(Ti*Td), N = 10, b = 1, umin = -Inf, umax = Inf, Ts, I = 0, D = 0, yold = 0)

A discrete-time PID controller with set-point weighting and integrator anti-windup.
The controller is implemented on the standard form
```math
u = K \\left( e + \\dfrac{1}{Ti} \\int e dt + T_d \\dfrac{de}{dt} \\right)
```

```math
U(s) = K \\left( bR(s) - Y(s) + \\dfrac{1}{sT_i} \\left( R(s) Y(s) \\right) - \\dfrac{sT_d}{1 + s T_d / N}Y(s)
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
- `b`: Fraction of set point in proportional term
- `umin`: Low output limit
- `umax`: High output limit
- `Ts`: Sampling period
- `I`: Integral part
- `D`: Derivative part
- `yold`: Last measurement signal

See also [`calculate_control!`](@ref), [`set_K!`](@ref), [`set_Ti!`](@ref), [`set_Td!`](@ref)
"""
function DiscretePID(;
    K::T  = 1f0,
    Ti = false,
    Td = false,
    Tt = Ti > 0 && Td > 0 ? floattype(K)(√(Ti*Td)) : floattype(K)(10),
    N  = floattype(K)(10),
    b  = floattype(K)(1),
    umin = typemin(floattype(K)),
    umax = typemax(floattype(K)),
    Ts,
    I    = zero(floattype(K)),
    D    = zero(floattype(K)),
    yold = zero(floattype(K)),
) where T
    if Ti > 0
        bi = K * Ts / Ti
    else
        bi = zero(K * Ts)
    end
    Tt ≥ 0 || throw(ArgumentError("Tt must be positive"))
    Td ≥ 0 || throw(ArgumentError("Td must be positive"))
    N ≥ 0 || throw(ArgumentError("N must be positive"))
    0 ≤ b ≤ 1 || throw(ArgumentError("b must be ∈ [0, 1]"))
    umax > umin || throw(ArgumentError("umax must be greater than umin"))

    if Ti > 0
        ar = Ts / Tt
    else
        ar = zero(Ts / Tt)
    end
    ad = Td / (Td + N * Ts)
    bd = K * N * ad

    T2 = promote_type(typeof.((K, Ti, Td, Tt, N, b, umin, umax, Ts, bi, ar, bd, ad, I, D, yold))...)

    DiscretePID(T2.((K, Ti, Td, Tt, N, b, umin, umax, Ts, bi, ar, bd, ad, I, D, yold))...)
end

"""
    set_K!(pid::DiscretePID, K, r, y)

Update `K` in the PID controller. This function takes the current reference and measurement as well in order to provide bumpless transfer. This is realized by updating the internal state `I`.
"""
function set_K!(pid::DiscretePID, K, r, y)
    Kold = pid.K
    pid.K = K
    pid.bd = K * pid.N * pid.ad
    if pid.Ti > 0
        pid.bi = K * pid.Ts / pid.Ti
        pid.I = pid.I + Kold*(pid.b*r - y) - K*(pid.b*r - y)
    end
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
end


"""
    u = calculate_control!(pid::DiscretePID, r, y, uff=0)
    (pid)(r, y, uff=0) # Alternative syntax

Calculate the control output from the PID controller when `r` is the reference (set point), `y` is the latest measurement and `uff` is the feed-forward contribution.
If the type of the input arguments differ from the numeric type used by the PID controller, they will be converted before computations are performed.
"""
function calculate_control!(pid::DiscretePID{T}, r0, y0, uff0=0) where T
    r = T(r0)
    y = T(y0)
    uff = T(uff0)
    P = pid.K * (pid.b * r - y)
    pid.D = pid.ad * pid.D - pid.bd * (y - pid.yold)
    v = P + pid.I + pid.D + uff
    u = clamp(v, pid.umin, pid.umax)
    pid.I = pid.I + pid.bi * (r - y) + pid.ar * (u - v)
    pid.yold = y
    return u
end

(pid::DiscretePID)(args...) = calculate_control!(pid, args...)

function Base.show(io::IO, ::MIME"text/plain", pid::DiscretePID)
    println(io, "$(typeof(pid))( # with parameters and state:")
    for name in fieldnames(DiscretePID)
        @printf(io, "    %-12.8g,# %s\n", getfield(pid, name), name)
    end
    println(io, ")")
end


end
