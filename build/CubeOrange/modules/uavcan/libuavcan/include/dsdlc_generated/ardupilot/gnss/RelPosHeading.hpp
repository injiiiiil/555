/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/jon305/ardupilot/modules/DroneCAN/DSDL/ardupilot/gnss/20006.RelPosHeading.uavcan
 */

#ifndef ARDUPILOT_GNSS_RELPOSHEADING_HPP_INCLUDED
#define ARDUPILOT_GNSS_RELPOSHEADING_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

#include <uavcan/Timestamp.hpp>

/******************************* Source text **********************************
# timestamp on the gps message
uavcan.Timestamp timestamp

bool    reported_heading_acc_available
float32 reported_heading_deg
float32 reported_heading_acc_deg
float16 relative_distance_m
float16 relative_down_pos_m
******************************************************************************/

/********************* DSDL signature source definition ***********************
ardupilot.gnss.RelPosHeading
uavcan.Timestamp timestamp
saturated bool reported_heading_acc_available
saturated float32 reported_heading_deg
saturated float32 reported_heading_acc_deg
saturated float16 relative_distance_m
saturated float16 relative_down_pos_m
******************************************************************************/

#undef timestamp
#undef reported_heading_acc_available
#undef reported_heading_deg
#undef reported_heading_acc_deg
#undef relative_distance_m
#undef relative_down_pos_m

namespace ardupilot
{
namespace gnss
{

template <int _tmpl>
struct UAVCAN_EXPORT RelPosHeading_
{
    typedef const RelPosHeading_<_tmpl>& ParameterType;
    typedef RelPosHeading_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Timestamp timestamp;
        typedef ::uavcan::IntegerSpec< 1, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > reported_heading_acc_available;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > reported_heading_deg;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > reported_heading_acc_deg;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > relative_distance_m;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > relative_down_pos_m;
    };

    enum
    {
        MinBitLen
            = FieldTypes::timestamp::MinBitLen
            + FieldTypes::reported_heading_acc_available::MinBitLen
            + FieldTypes::reported_heading_deg::MinBitLen
            + FieldTypes::reported_heading_acc_deg::MinBitLen
            + FieldTypes::relative_distance_m::MinBitLen
            + FieldTypes::relative_down_pos_m::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::timestamp::MaxBitLen
            + FieldTypes::reported_heading_acc_available::MaxBitLen
            + FieldTypes::reported_heading_deg::MaxBitLen
            + FieldTypes::reported_heading_acc_deg::MaxBitLen
            + FieldTypes::relative_distance_m::MaxBitLen
            + FieldTypes::relative_down_pos_m::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::timestamp >::Type timestamp;
    typename ::uavcan::StorageType< typename FieldTypes::reported_heading_acc_available >::Type reported_heading_acc_available;
    typename ::uavcan::StorageType< typename FieldTypes::reported_heading_deg >::Type reported_heading_deg;
    typename ::uavcan::StorageType< typename FieldTypes::reported_heading_acc_deg >::Type reported_heading_acc_deg;
    typename ::uavcan::StorageType< typename FieldTypes::relative_distance_m >::Type relative_distance_m;
    typename ::uavcan::StorageType< typename FieldTypes::relative_down_pos_m >::Type relative_down_pos_m;

    RelPosHeading_()
        : timestamp()
        , reported_heading_acc_available()
        , reported_heading_deg()
        , reported_heading_acc_deg()
        , relative_distance_m()
        , relative_down_pos_m()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<153 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 20006 };

    static const char* getDataTypeFullName()
    {
        return "ardupilot.gnss.RelPosHeading";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool RelPosHeading_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        timestamp == rhs.timestamp &&
        reported_heading_acc_available == rhs.reported_heading_acc_available &&
        reported_heading_deg == rhs.reported_heading_deg &&
        reported_heading_acc_deg == rhs.reported_heading_acc_deg &&
        relative_distance_m == rhs.relative_distance_m &&
        relative_down_pos_m == rhs.relative_down_pos_m;
}

template <int _tmpl>
bool RelPosHeading_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(timestamp, rhs.timestamp) &&
        ::uavcan::areClose(reported_heading_acc_available, rhs.reported_heading_acc_available) &&
        ::uavcan::areClose(reported_heading_deg, rhs.reported_heading_deg) &&
        ::uavcan::areClose(reported_heading_acc_deg, rhs.reported_heading_acc_deg) &&
        ::uavcan::areClose(relative_distance_m, rhs.relative_distance_m) &&
        ::uavcan::areClose(relative_down_pos_m, rhs.relative_down_pos_m);
}

template <int _tmpl>
int RelPosHeading_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::timestamp::encode(self.timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::reported_heading_acc_available::encode(self.reported_heading_acc_available, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::reported_heading_deg::encode(self.reported_heading_deg, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::reported_heading_acc_deg::encode(self.reported_heading_acc_deg, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::relative_distance_m::encode(self.relative_distance_m, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::relative_down_pos_m::encode(self.relative_down_pos_m, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RelPosHeading_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::timestamp::decode(self.timestamp, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::reported_heading_acc_available::decode(self.reported_heading_acc_available, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::reported_heading_deg::decode(self.reported_heading_deg, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::reported_heading_acc_deg::decode(self.reported_heading_acc_deg, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::relative_distance_m::decode(self.relative_distance_m, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::relative_down_pos_m::decode(self.relative_down_pos_m, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature RelPosHeading_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0xB2F757F09F08BCD0ULL);

    FieldTypes::timestamp::extendDataTypeSignature(signature);
    FieldTypes::reported_heading_acc_available::extendDataTypeSignature(signature);
    FieldTypes::reported_heading_deg::extendDataTypeSignature(signature);
    FieldTypes::reported_heading_acc_deg::extendDataTypeSignature(signature);
    FieldTypes::relative_distance_m::extendDataTypeSignature(signature);
    FieldTypes::relative_down_pos_m::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef RelPosHeading_<0> RelPosHeading;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::ardupilot::gnss::RelPosHeading > _uavcan_gdtr_registrator_RelPosHeading;

}

} // Namespace gnss
} // Namespace ardupilot

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::ardupilot::gnss::RelPosHeading >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::ardupilot::gnss::RelPosHeading::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::ardupilot::gnss::RelPosHeading >::stream(Stream& s, ::ardupilot::gnss::RelPosHeading::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "timestamp: ";
    YamlStreamer< ::ardupilot::gnss::RelPosHeading::FieldTypes::timestamp >::stream(s, obj.timestamp, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "reported_heading_acc_available: ";
    YamlStreamer< ::ardupilot::gnss::RelPosHeading::FieldTypes::reported_heading_acc_available >::stream(s, obj.reported_heading_acc_available, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "reported_heading_deg: ";
    YamlStreamer< ::ardupilot::gnss::RelPosHeading::FieldTypes::reported_heading_deg >::stream(s, obj.reported_heading_deg, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "reported_heading_acc_deg: ";
    YamlStreamer< ::ardupilot::gnss::RelPosHeading::FieldTypes::reported_heading_acc_deg >::stream(s, obj.reported_heading_acc_deg, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "relative_distance_m: ";
    YamlStreamer< ::ardupilot::gnss::RelPosHeading::FieldTypes::relative_distance_m >::stream(s, obj.relative_distance_m, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "relative_down_pos_m: ";
    YamlStreamer< ::ardupilot::gnss::RelPosHeading::FieldTypes::relative_down_pos_m >::stream(s, obj.relative_down_pos_m, level + 1);
}

}

namespace ardupilot
{
namespace gnss
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::ardupilot::gnss::RelPosHeading::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::ardupilot::gnss::RelPosHeading >::stream(s, obj, 0);
    return s;
}

} // Namespace gnss
} // Namespace ardupilot

#endif // ARDUPILOT_GNSS_RELPOSHEADING_HPP_INCLUDED