module JuliacPID
import DiscretePIDs
import DiscretePIDs: DiscretePID
import Base.@ccallable

const T = Float64 # The numeric type used by the controller

# Set the initial PID parameters here
const pid = DiscretePIDs.DiscretePID(; K = T(1), Ti = 1, Td = false, Ts = 1)


@ccallable function calculate_control!(r::T, y::T, uff::T)::T
    DiscretePIDs.calculate_control!(pid, r, y, uff)::T
end

@ccallable function set_K!(K::T, r::T, y::T)::Cvoid
    DiscretePIDs.set_K!(pid, K, r, y)
    nothing
end

@ccallable function set_Ti!(Ti::T)::Cvoid
    DiscretePIDs.set_Ti!(pid, Ti)
    nothing
end

@ccallable function set_Td!(Td::T)::Cvoid
    DiscretePIDs.set_Td!(pid, Td)
    nothing
end

@ccallable function reset_state!()::Cvoid
    DiscretePIDs.reset_state!(pid)
    nothing
end

# @ccallable function main()::Cint
#     println(Core.stdout, "I'm alive and well")
#     u = calculate_control!(0.0, 0.0, 0.0)
#     println(Core.stdout, u)

#     Cint(0)
# end


end

# compile using something like this, modified to suit your local paths
# cd(@__DIR__)
# run(`/home/fredrikb/repos/julia/julia --project --experimental /home/fredrikb/repos/julia/contrib/juliac.jl --output-lib juliac_pid --trim=unsafe-warn --experimental --compile-ccallable juliac_pid.jl`)
# run(`ls -ltrh`)
