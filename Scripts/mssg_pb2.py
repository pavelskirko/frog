# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: mssg.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='mssg.proto',
  package='',
  syntax='proto3',
  serialized_options=None,
  serialized_pb=_b('\n\nmssg.proto\"Z\n\x05\x41\x63\x63\x65l\x12\x0b\n\x03\x61_x\x18\x01 \x01(\r\x12\x0b\n\x03\x61_y\x18\x02 \x01(\r\x12\x0b\n\x03\x61_z\x18\x03 \x01(\r\x12\x0c\n\x04time\x18\x04 \x01(\x04\x12\n\n\x02up\x18\x05 \x01(\x08\x12\x10\n\x08last_msg\x18\x06 \x01(\x08\"R\n\x0b\x46inalResult\x12\x12\n\x02up\x18\x01 \x03(\x0b\x32\x06.Accel\x12\x14\n\x04\x64own\x18\x02 \x03(\x0b\x32\x06.Accel\x12\x0c\n\x04tens\x18\x03 \x03(\x05\x12\x0b\n\x03log\x18\x04 \x03(\tb\x06proto3')
)




_ACCEL = _descriptor.Descriptor(
  name='Accel',
  full_name='Accel',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='a_x', full_name='Accel.a_x', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='a_y', full_name='Accel.a_y', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='a_z', full_name='Accel.a_z', index=2,
      number=3, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='time', full_name='Accel.time', index=3,
      number=4, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='up', full_name='Accel.up', index=4,
      number=5, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='last_msg', full_name='Accel.last_msg', index=5,
      number=6, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=14,
  serialized_end=104,
)


_FINALRESULT = _descriptor.Descriptor(
  name='FinalResult',
  full_name='FinalResult',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='up', full_name='FinalResult.up', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='down', full_name='FinalResult.down', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tens', full_name='FinalResult.tens', index=2,
      number=3, type=5, cpp_type=1, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='log', full_name='FinalResult.log', index=3,
      number=4, type=9, cpp_type=9, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=106,
  serialized_end=188,
)

_FINALRESULT.fields_by_name['up'].message_type = _ACCEL
_FINALRESULT.fields_by_name['down'].message_type = _ACCEL
DESCRIPTOR.message_types_by_name['Accel'] = _ACCEL
DESCRIPTOR.message_types_by_name['FinalResult'] = _FINALRESULT
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Accel = _reflection.GeneratedProtocolMessageType('Accel', (_message.Message,), dict(
  DESCRIPTOR = _ACCEL,
  __module__ = 'mssg_pb2'
  # @@protoc_insertion_point(class_scope:Accel)
  ))
_sym_db.RegisterMessage(Accel)

FinalResult = _reflection.GeneratedProtocolMessageType('FinalResult', (_message.Message,), dict(
  DESCRIPTOR = _FINALRESULT,
  __module__ = 'mssg_pb2'
  # @@protoc_insertion_point(class_scope:FinalResult)
  ))
_sym_db.RegisterMessage(FinalResult)


# @@protoc_insertion_point(module_scope)
