## Using ProtoBuf

Reading and writing data structures using ProtoBuf is similar to serialization and deserialization. Methods `writeproto` and `readproto` can write and read Julia types from IO streams.

````
julia> using ProtoBuf                       # include protoc generated package here

julia> mutable struct MyType                # a Julia composite type generated from protoc
         intval::Int
         strval::String
         MyType(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
       end

julia> iob = PipeBuffer();

julia> writeproto(iob, MyType(intval=10, strval="hello world"));   # write an instance of it

julia> readproto(iob, MyType())  # read it back into another instance
MyType(10,"hello world")
````

## Protocol Buffer Metadata

ProtoBuf serialization can be customized for a type by defining a `meta` method on it. The `meta` method provides an instance of `ProtoMeta` that allows specification of mandatory fields, field numbers, and default values for fields for a type. Defining a specialized `meta` is done simply as below:

````
import ProtoBuf.meta

meta(t::Type{MyType}) = meta(t,                          # the type which this is for
		Symbol[:intval],                                 # required fields
		Int[8, 10],                                      # field numbers
		Dict{Symbol,Any}(:strval => "default value"))  # default values
````

Without any specialized `meta` method:

- All fields are marked as optional (or repeating for arrays)
- Numeric fields have zero as default value
- String fields have `""` as default value
- Field numbers are assigned serially starting from 1, in the order of their declaration.

For the things where the default is what you need, just passing empty values would do. E.g., if you just want to specify the field numbers, this would do:

````
meta(t::Type{MyType}) = meta(t, [], [8,10], Dict())
````

## Setting and Getting Fields
Types used as protocol buffer structures are regular Julia types and the Julia syntax to set and get fields can be used on them. But with fields that are set as optional, it is quite likely that some of them may not have been present in the instance that was read. The following methods are exported to assist doing this:

- `get_field(obj::Any, fld::Symbol)` : Gets `obj.fld` if it has been set. Throws an error otherwise.
- `has_field(obj::Any, fld::Symbol)` : Checks whether field `fld` has been set in `obj`.
- `clear(obj::Any, fld::Symbol)` : Marks field `fld` of `obj` as unset.
- `clear(obj::Any)` : Marks all fields of `obj` as unset.

The `protobuild` method makes it easier to set large types with many fields:
- `protobuild{T}(::Type{T}, nvpairs::Dict{Symbol}()=Dict{Symbol,Any}())`

Types generated through the Julia protoc plugin generates constructors that use `protobuild` and expect keyword arguments for the type members.

````
julia> using ProtoBuf

julia> mutable struct MyType <: ProtoType  # a Julia composite type
           intval::Int
           # fillunset (documented below is similar to clear)
           # ProtoBuf._protobuild is an internal method similar to protobuild
           MyType(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
       end

julia> mutable struct OptType <: ProtoType # and another one to contain it
           opt::MyType
           OptType(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
       end

julia> iob = PipeBuffer();

julia> writeproto(iob, OptType(opt=MyType(intval=10)));

julia> readval = readproto(iob, OptType());

julia> has_field(readval, :opt)       # valid this time
true

julia> writeproto(iob, OptType());

julia> readval = readproto(iob, OptType());

julia> has_field(readval, :opt)       # but not valid now
false
````

Note: The constructor for types generated by the `protoc` compiler have a call to `clear` to mark all fields of the object as unset to start with. A similar call must be made explicitly while using Julia types that are not generated. Otherwise any defined field in an instance is assumed to be valid.


The `isinitialized(obj::Any)` method checks whether all mandatory fields are set. It is useful to check objects using this method before sending them. Method `writeproto` results in an exception if this condition is violated.

````
julia> using ProtoBuf

julia> import ProtoBuf.meta

julia> mutable struct TestType <: ProtoType
           val::Any
       end

julia> mutable struct TestFilled <: ProtoType
           fld1::TestType
           fld2::TestType
           TestFilled(; kwargs...) = (o=new(); fillunset(o); isempty(kwargs) || ProtoBuf._protobuild(o, kwargs); o)
       end

julia> meta(t::Type{TestFilled}) = meta(t, Symbol[:fld1], Int[], Dict{Symbol,Any}());

julia> tf = TestFilled()
TestFilled(#undef,#undef)

julia> isinitialized(tf)      # false, since fld1 is not set
false

julia> tf.fld1 = TestType("");

julia> isinitialized(tf)      # true, even though fld2 is not set yet
true
````

## Equality &amp; Hash Value
It is possible for fields marked as optional to be in an &quot;unset&quot; state. Even bits type fields (`isbitstype(T) == true`) can be in this state though they may have valid contents. Such fields should then not be compared for equality or used for computing hash values. All ProtoBuf compatible types, by virtue of extending abstract `ProtoType` type, override `hash`, `isequal` and `==` methods to handle this. The following unexported utility methods can be used for this purpose, in cases where it is not possible to extend `ProtoType`:

- `protohash(v)` : hash method that considers fill status of types
- `protoeq{T}(v1::T, v2::T)` : equality method that considers fill status of types
- `protoisequal{T}(v1::T, v2::T)` : isequal method that considers fill status of types

## Other Methods
- `copy!{T}(to::T, from::T)` : shallow copy of objects
- `isfilled(obj, fld::Symbol)` : same as `has_field`
- `isfilled(obj)` : same as `isinitialized`
- `isfilled_default(obj, fld::Symbol)` : whether field is set with default value (and not deserialized)
- `fillset(obj, fld::Symbol)` : mark field fld of object obj as set
- `fillunset(obj)` : mark all fields of this object as not set
- `fillunset(obj::Any, fld::Symbol)` : mark field fld of object obj as not set
- `lookup(en::ProtoEnum,val::Integer)` : lookup the name (symbol) corresponding to an enum value
- `enumstr(enumname, enumvalue::Int32)`: returns a string with the enum field name matching the value
- `which_oneof(obj, oneof::Symbol)`: returns a symbol indicating the name of the field in the `oneof` group that is filled

