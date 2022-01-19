/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/jon305/ardupilot/modules/DroneCAN/DSDL/uavcan/equipment/air_data/1026.Sideslip.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_AIR_DATA_SIDESLIP_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_AIR_DATA_SIDESLIP_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Body sideslip in radians.
#

float16 sideslip_angle          # Radians
float16 sideslip_angle_variance # Radians^2
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.air_data.Sideslip
saturated float16 sideslip_angle
saturated float16 sideslip_angle_variance
******************************************************************************/

#undef sideslip_angle
#undef sideslip_angle_variance

namespace uavcan
{
namespace equipment
{
namespace air_data
{

template <int _tmpl>
struct UAVCAN_EXPORT Sideslip_
{
    typedef const Sideslip_<_tmpl>& ParameterType;
    typedef Sideslip_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > sideslip_angle;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > sideslip_angle_variance;
    };

    enum
    {
        MinBitLen
            = FieldTypes::sideslip_angle::MinBitLen
            + FieldTypes::sideslip_angle_variance::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::sideslip_angle::MaxBitLen
            + FieldTypes::sideslip_angle_variance::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::sideslip_angle >::Type sideslip_angle;
    typename ::uavcan::StorageType< typename FieldTypes::sideslip_angle_variance >::Type sideslip_angle_variance;

    Sideslip_()
        : sideslip_angle()
        , sideslip_angle_variance()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<32 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1026 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.air_data.Sideslip";
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
bool Sideslip_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        sideslip_angle == rhs.sideslip_angle &&
        sideslip_angle_variance == rhs.sideslip_angle_variance;
}

template <int _tmpl>
bool Sideslip_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(sideslip_angle, rhs.sideslip_angle) &&
        ::uavcan::areClose(sideslip_angle_variance, rhs.sideslip_angle_variance);
}

template <int _tmpl>
int Sideslip_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::sideslip_angle::encode(self.sideslip_angle, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::sideslip_angle_variance::encode(self.sideslip_angle_variance, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int Sideslip_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::sideslip_angle::decode(self.sideslip_angle, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::sideslip_angle_variance::decode(self.sideslip_angle_variance, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature Sideslip_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x7B48E55FCFF42A57ULL);

    FieldTypes::sideslip_angle::extendDataTypeSignature(signature);
    FieldTypes::sideslip_angle_variance::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef Sideslip_<0> Sideslip;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::air_data::Sideslip > _uavcan_gdtr_registrator_Sideslip;

}

} // Namespace air_data
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::air_data::Sideslip >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::air_data::Sideslip::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::air_data::Sideslip >::stream(Stream& s, ::uavcan::equipment::air_data::Sideslip::ParameterType obj, const int level)
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
    s << "sideslip_angle: ";
    YamlStreamer< ::uavcan::equipment::air_data::Sideslip::FieldTypes::sideslip_angle >::stream(s, obj.sideslip_angle, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "sideslip_angle_variance: ";
    YamlStreamer< ::uavcan::equipment::air_data::Sideslip::FieldTypes::sideslip_angle_variance >::stream(s, obj.sideslip_angle_variance, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace air_data
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::air_data::Sideslip::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::air_data::Sideslip >::stream(s, obj, 0);
    return s;
}

} // Namespace air_data
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_AIR_DATA_SIDESLIP_HPP_INCLUDED