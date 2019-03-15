using ProtoBuf
include("mssg_pb.jl")

file = open("result_with_gyro_2.txt", "r")
a = Accel()
readproto(file, a)
print(a)
