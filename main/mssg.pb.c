/* Automatically generated nanopb constant definitions */
/* Generated by nanopb-0.3.9.2 at Fri Feb 01 21:42:27 2019. */

#include "mssg.pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif



const pb_field_t Accel_fields[4] = {
    PB_FIELD(  1, INT32   , SINGULAR, STATIC  , FIRST, Accel, a_x, a_x, 0),
    PB_FIELD(  2, INT32   , SINGULAR, STATIC  , OTHER, Accel, a_y, a_x, 0),
    PB_FIELD(  3, INT32   , SINGULAR, STATIC  , OTHER, Accel, a_z, a_y, 0),
    PB_LAST_FIELD
};

const pb_field_t FinalResult_fields[5] = {
    PB_FIELD(  1, MESSAGE , REPEATED, CALLBACK, FIRST, FinalResult, up, up, &Accel_fields),
    PB_FIELD(  2, MESSAGE , REPEATED, CALLBACK, OTHER, FinalResult, down, up, &Accel_fields),
    PB_FIELD(  3, INT32   , REPEATED, CALLBACK, OTHER, FinalResult, tens, down, 0),
    PB_FIELD(  4, STRING  , REPEATED, CALLBACK, OTHER, FinalResult, log, tens, 0),
    PB_LAST_FIELD
};


/* @@protoc_insertion_point(eof) */
