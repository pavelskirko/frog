# syntax: proto3
using ProtoBuf
import ProtoBuf.meta

mutable struct Accel <: ProtoType
    a_x::Int32
    a_y::Int32
    a_z::Int32
    time::UInt32
    up::Bool
    last_msg::Bool
    number::UInt32
    g_x::Int32
    g_y::Int32
    g_z::Int32
    Accel(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
end #mutable struct Accel

mutable struct FinalResult <: ProtoType
    up::Base.Vector{Accel}
    down::Base.Vector{Accel}
    tens::Base.Vector{Int32}
    log::Base.Vector{AbstractString}
    FinalResult(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
end #mutable struct FinalResult
const __pack_FinalResult = Symbol[:tens]
meta(t::Type{FinalResult}) = meta(t, ProtoBuf.DEF_REQ, ProtoBuf.DEF_FNUM, ProtoBuf.DEF_VAL, true, __pack_FinalResult, ProtoBuf.DEF_WTYPES, ProtoBuf.DEF_ONEOFS, ProtoBuf.DEF_ONEOF_NAMES, ProtoBuf.DEF_FIELD_TYPES)

export Accel, FinalResult
