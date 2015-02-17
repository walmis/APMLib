
#ifndef __AP_HAL_SEMAPHORES_H__
#define __AP_HAL_SEMAPHORES_H__

#include <AP_HAL_Namespace.h>

#define HAL_SEMAPHORE_BLOCK_FOREVER ((uint32_t) 0xFFFFFFFF)

class AP_HAL::Semaphore {
public:
    virtual bool take(uint32_t timeout_ms) WARN_IF_UNUSED = 0 ;
    virtual bool take_nonblocking() WARN_IF_UNUSED = 0;
    /* Take semaphore asynchronously
     * if semaphore is locked, this function will return false
     * and callback will be called when semaphore is released.
     * The callback function should check the semaphore again.
     * But if semaphore is unlocked, then this function will return true
     * and the callback will not be executed.
     * Impl: Callback should not be called from interrupt context.
     */
    virtual bool take_async(AP_HAL::MemberProc callback) WARN_IF_UNUSED = 0;
    virtual bool give() = 0;
};

#endif  // __AP_HAL_SEMAPHORES_H__
