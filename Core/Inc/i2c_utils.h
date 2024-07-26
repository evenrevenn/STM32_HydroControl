#ifndef I2C_UTILS
#define I2C_UTILS

#include "allocator.h"

#include <cstdint>
#include <functional>
#include <map>

template <uint8_t id, uint8_t args_num, bool write_follows> requires (id <= 0x1F) && (args_num <= 0x03)
struct _generateCommandId;

#define RESERVE_I2C_ID(name, id, args_num, write_follows)   template<> \
                                                    		struct _generateCommandId<id, args_num, write_follows> \
                                                    		{ enum {value = (write_follows ? 0x80 : 0x00) | (args_num & 0x03) << 5 | (id & 0x1F)}; }; \
															typedef _generateCommandId<id, args_num, write_follows> _i2c_##name##_id;

#define I2C_CMD_DECLARE(name) void _i2c_##name(I2C_Args args);
#define I2C_CMD_INIT(name, class_name, handler_ref) handler_ref.addCallback((uint8_t)_i2c_##name##_id::value, std::bind(&class_name::_i2c_##name, this, std::placeholders::_1));

typedef uint8_t I2C_CommandId;


struct I2C_Function
{
	const uint32_t args = 0;
	const std::function<void(uint32_t)> function;
};

typedef std::map<I2C_CommandId,
				std::function<void(uint32_t)>,
        		std::less<I2C_CommandId>,
        		Heap4Alocator< std::map<I2C_CommandId, std::function<void(uint32_t)>>::value_type >
				> I2C_CallbackMap;

class I2C_Args
{
public:
	I2C_Args(uint32_t args):args(args){}

	template<unsigned int offset>
	const uint8_t getByte() const {static_assert(offset < 4); return (args >> 8 * offset) & 0xFF;}

	template<unsigned int offset>
	const uint16_t getShort() const {static_assert(offset < 3); return (args >> 8 * offset) & 0xFFFF;}

	const uint32_t getWord() const {return args;}

private:
	const uint32_t args;
};

#endif //I2C_UTILS