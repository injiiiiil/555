/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/jon305/ardupilot/modules/DroneCAN/DSDL/uavcan/equipment/esc/1030.RawCommand.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Raw ESC command normalized into [-8192, 8191]; negative values indicate reverse rotation.
# The ESC should normalize the setpoint into its effective input range.
# Non-zero setpoint value below minimum should be interpreted as min valid setpoint for the given motor.
#

int14[<=20] cmd
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.esc.RawCommand
saturated int14[<=20] cmd
******************************************************************************/

#undef cmd

namespace uavcan
{
namespace equipment
{
namespace esc
{

template <int _tmpl>
struct UAVCAN_EXPORT RawCommand_
{
    typedef const RawCommand_<_tmpl>& ParameterType;
    typedef RawCommand_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::Array< ::uavcan::IntegerSpec< 14, ::uavcan::SignednessSigned, ::uavcan::CastModeSaturate >, ::uavcan::ArrayModeDynamic, 20 > cmd;
    };

    enum
    {
        MinBitLen
            = FieldTypes::cmd::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::cmd::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::cmd >::Type cmd;

    RawCommand_()
        : cmd()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<285 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1030 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.esc.RawCommand";
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
bool RawCommand_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        cmd == rhs.cmd;
}

template <int _tmpl>
bool RawCommand_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(cmd, rhs.cmd);
}

template <int _tmpl>
int RawCommand_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::cmd::encode(self.cmd, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RawCommand_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::cmd::decode(self.cmd, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature RawCommand_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x217F5C87D7EC951DULL);

    FieldTypes::cmd::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef RawCommand_<0> RawCommand;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::esc::RawCommand > _uavcan_gdtr_registrator_RawCommand;

}

} // Namespace esc
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::esc::RawCommand >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::esc::RawCommand::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::esc::RawCommand >::stream(Stream& s, ::uavcan::equipment::esc::RawCommand::ParameterType obj, const int level)
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
    s << "cmd: ";
    YamlStreamer< ::uavcan::equipment::esc::RawCommand::FieldTypes::cmd >::stream(s, obj.cmd, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace esc
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::esc::RawCommand::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::esc::RawCommand >::stream(s, obj, 0);
    return s;
}

} // Namespace esc
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_HPP_INCLUDED