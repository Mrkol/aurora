#include "mtx_assert.h"
#include <stdint.h>
#include <dolphin/mtx.h>

void MTXInitStack(MTXStack* sPtr, u32 numMtx) {
    MTX_ASSERT(sPtr && "MTXInitStack():  NULL MtxStackPtr 'sPtr' ");
    MTX_ASSERT(sPtr->stackBase && "MTXInitStack():  'sPtr' contains a NULL ptr to stack memory ");
    MTX_ASSERT(numMtx && "MTXInitStack():  'numMtx' is 0 ");

    sPtr->numMtx = numMtx;
    sPtr->stackPtr = 0;
}

MtxPtr MTXPush(MTXStack* sPtr, const Mtx m) {
    MTX_ASSERT(sPtr && "MTXPush():  NULL MtxStackPtr 'sPtr' ");
    MTX_ASSERT(sPtr->stackBase && "MTXPush():  'sPtr' contains a NULL ptr to stack memory ");
    MTX_ASSERT(m && "MTXPush():  NULL MtxPtr 'm' ");

    if (sPtr->stackPtr == NULL) {
        sPtr->stackPtr = sPtr->stackBase;
        MTXCopy(m, sPtr->stackPtr);
    } else {
        MTX_ASSERT(((((uintptr_t)sPtr->stackPtr - (uintptr_t)sPtr->stackBase) / 16) / 3) < (sPtr->numMtx - 1) && "MTXPush():  stack overflow ");
        MTXCopy(m, sPtr->stackPtr + 3);
        sPtr->stackPtr += 3;
    }

    return sPtr->stackPtr;
}

MtxPtr MTXPushFwd(MTXStack* sPtr, const Mtx m) {
    MTX_ASSERT(sPtr && "MTXPushFwd():  NULL MtxStackPtr 'sPtr' ");
    MTX_ASSERT(sPtr->stackBase && "MTXPushFwd():  'sPtr' contains a NULL ptr to stack memory ");
    MTX_ASSERT(m && "MTXPushFwd():  NULL MtxPtr 'm' ");

    if (sPtr->stackPtr == NULL) {
        sPtr->stackPtr = sPtr->stackBase;
        MTXCopy(m, sPtr->stackPtr);
    } else {
        MTX_ASSERT(((((uintptr_t)sPtr->stackPtr - (uintptr_t)sPtr->stackBase) / 16) / 3) < (sPtr->numMtx - 1) && "MTXPushFwd():  stack overflow");
        MTXConcat(sPtr->stackPtr, m, sPtr->stackPtr + 3);
        sPtr->stackPtr += 3;
    }

    return sPtr->stackPtr;
}

MtxPtr MTXPushInv(MTXStack* sPtr, const Mtx m) {
    Mtx mInv;
    MTX_ASSERT(sPtr && "MTXPushInv():  NULL MtxStackPtr 'sPtr' ");
    MTX_ASSERT(sPtr->stackBase && "MTXPushInv():  'sPtr' contains a NULL ptr to stack memory ");
    MTX_ASSERT(m && "MTXPushInv():  NULL MtxPtr 'm' ");

    MTXInverse(m, mInv);
    if (sPtr->stackPtr == NULL) {
        sPtr->stackPtr = sPtr->stackBase;
        MTXCopy(mInv, sPtr->stackPtr);
    } else {
        MTX_ASSERT(((((uintptr_t)sPtr->stackPtr - (uintptr_t)sPtr->stackBase) / 16) / 3) < (sPtr->numMtx - 1) && "MTXPushInv():  stack overflow");
        MTXConcat(mInv, sPtr->stackPtr, sPtr->stackPtr + 3);
        sPtr->stackPtr += 3;
    }

    return sPtr->stackPtr;
}

MtxPtr MTXPushInvXpose(MTXStack* sPtr, const Mtx m) {
    Mtx mIT;
    MTX_ASSERT(sPtr && "MTXPushInvXpose():  NULL MtxStackPtr 'sPtr' ");
    MTX_ASSERT(sPtr->stackBase && "MTXPushInvXpose():  'sPtr' contains a NULL ptr to stack memory ");
    MTX_ASSERT(m && "MTXPushInvXpose():  NULL MtxPtr 'm' ");

    MTXInverse(m, mIT);
    MTXTranspose(mIT, mIT);
    if (sPtr->stackPtr == NULL) {
        sPtr->stackPtr = sPtr->stackBase;
        MTXCopy(mIT, sPtr->stackPtr);
    } else {
        MTX_ASSERT(((((uintptr_t)sPtr->stackPtr - (uintptr_t)sPtr->stackBase) / 16) / 3) < (sPtr->numMtx - 1) && "MTXPushInvXpose():  stack overflow ");
        MTXConcat(sPtr->stackPtr, mIT, sPtr->stackPtr + 3);
        sPtr->stackPtr += 3;
    }

    return sPtr->stackPtr;
}

MtxPtr MTXPop(MTXStack* sPtr) {
    MTX_ASSERT(sPtr && "MTXPop():  NULL MtxStackPtr 'sPtr' ");
    MTX_ASSERT(sPtr->stackBase && "MTXPop():  'sPtr' contains a NULL ptr to stack memory ");
    
    if (sPtr->stackPtr == NULL) {
        return NULL;
    }

    if (sPtr->stackBase == sPtr->stackPtr) {
        sPtr->stackPtr = NULL;
        return NULL;
    }

    sPtr->stackPtr -= 3;
    return sPtr->stackPtr;
}

MtxPtr MTXGetStackPtr(const MTXStack* sPtr) {
    MTX_ASSERT(sPtr && "MTXGetStackPtr():  NULL MtxStackPtr 'sPtr' ");
    MTX_ASSERT(sPtr->stackBase && "MTXGetStackPtr():  'sPtr' contains a NULL ptr to stack memory ");
    return sPtr->stackPtr;
}
