/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.2 at Mon Feb 04 05:43:02 2019. */

#ifndef PB_MSSG_PB_H_INCLUDED
#define PB_MSSG_PB_H_INCLUDED
#include "pb.h"

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _FinalResult {
    pb_callback_t up;
    pb_callback_t down;
    pb_callback_t tens;
    pb_callback_t log;
/* @@protoc_insertion_point(struct:FinalResult) */
} FinalResult;

typedef struct _Accel {
    uint32_t a_x;
    uint32_t a_y;
    uint32_t a_z;
    uint32_t time;
    bool up;
    bool last_msg;
/* @@protoc_insertion_point(struct:Accel) */
} Accel;

/* Default values for struct fields */

/* Initializer values for message structs */
#define Accel_init_default                       {0, 0, 0, 0, 0, 0}
#define FinalResult_init_default                 {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}
#define Accel_init_zero                          {0, 0, 0, 0, 0, 0}
#define FinalResult_init_zero                    {{{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}, {{NULL}, NULL}}

/* Field tags (for use in manual encoding/decoding) */
#define FinalResult_up_tag                       1
#define FinalResult_down_tag                     2
#define FinalResult_tens_tag                     3
#define FinalResult_log_tag                      4
#define Accel_a_x_tag                            1
#define Accel_a_y_tag                            2
#define Accel_a_z_tag                            3
#define Accel_time_tag                           4
#define Accel_up_tag                             5
#define Accel_last_msg_tag                       6

/* Struct field encoding specification for nanopb */
extern const pb_field_t Accel_fields[7];
extern const pb_field_t FinalResult_fields[5];

/* Maximum encoded size of messages (where known) */
#define Accel_size                               28
/* FinalResult_size depends on runtime parameters */

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define MSSG_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
