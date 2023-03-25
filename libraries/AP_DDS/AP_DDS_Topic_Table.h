// #include "Generated/Time.h" // might change to brackets for include path

#include "AP_DDS_Generic_Fn_T.h"

// Code generated table based on the enabled topics.
// Mavgen is using python, loops are not readable.
// Can use jinja to template (like Flask)


const struct AP_DDS_Client::Topic_table AP_DDS_Client::topics[] = {
    {
        .topic_id = 0x01,
        .pub_id = 0x01,
        .topic_profile_label = "time__t",
        .dw_profile_label = "time__dw",
        .serialize = Generic_serialize_topic_fn_t(&builtin_interfaces_msg_Time_serialize_topic),
        .deserialize = Generic_deserialize_topic_fn_t(&builtin_interfaces_msg_Time_deserialize_topic),
        .size_of = Generic_size_of_topic_fn_t(&builtin_interfaces_msg_Time_size_of_topic),
    },
};
