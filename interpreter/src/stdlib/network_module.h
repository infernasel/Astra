#ifndef ASTRA_STDLIB_NETWORK_MODULE_H
#define ASTRA_STDLIB_NETWORK_MODULE_H

#include "stdlib/network_module/network_module.h"
#include "runtime/runtime.h"

namespace astra {
namespace stdlib {

/**
 * @brief Register the network module with the runtime
 * 
 * @param runtime The runtime to register the module with
 */
void registerNetworkModule(runtime::Runtime& runtime);

} // namespace stdlib
} // namespace astra

#endif // ASTRA_STDLIB_NETWORK_MODULE_H