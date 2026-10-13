#include "mtx_assert.h"
#include <dolphin/mtx.h>

void C_MTXMultVec(const Mtx m, const Vec* src, Vec* dst) {
  Vec vTmp;

  MTX_ASSERT(m && "MTXMultVec():  NULL MtxPtr 'm' ");
  MTX_ASSERT(src && "MTXMultVec():  NULL VecPtr 'src' ");
  MTX_ASSERT(dst && "MTXMultVec():  NULL VecPtr 'dst' ");

  vTmp.x = m[0][3] + ((m[0][2] * src->z) + ((m[0][0] * src->x) + (m[0][1] * src->y)));
  vTmp.y = m[1][3] + ((m[1][2] * src->z) + ((m[1][0] * src->x) + (m[1][1] * src->y)));
  vTmp.z = m[2][3] + ((m[2][2] * src->z) + ((m[2][0] * src->x) + (m[2][1] * src->y)));
  dst->x = vTmp.x;
  dst->y = vTmp.y;
  dst->z = vTmp.z;
}

void C_MTXMultVecArray(const Mtx m, const Vec* srcBase, Vec* dstBase, u32 count) {
  u32 i;
  Vec vTmp;

  MTX_ASSERT(m && "MTXMultVecArray():  NULL MtxPtr 'm' ");
  MTX_ASSERT(srcBase && "MTXMultVecArray():  NULL VecPtr 'srcBase' ");
  MTX_ASSERT(dstBase && "MTXMultVecArray():  NULL VecPtr 'dstBase' ");
  MTX_ASSERT(count > 1 && "MTXMultVecArray():  count must be greater than 1.");

  for(i = 0; i < count; i++) {
    vTmp.x = m[0][3] + ((m[0][2] * srcBase->z) + ((m[0][0] * srcBase->x) + (m[0][1] * srcBase->y)));
    vTmp.y = m[1][3] + ((m[1][2] * srcBase->z) + ((m[1][0] * srcBase->x) + (m[1][1] * srcBase->y)));
    vTmp.z = m[2][3] + ((m[2][2] * srcBase->z) + ((m[2][0] * srcBase->x) + (m[2][1] * srcBase->y)));
    dstBase->x = vTmp.x;
    dstBase->y = vTmp.y;
    dstBase->z = vTmp.z;
    srcBase++;
    dstBase++;
  }
}

void C_MTXMultVecSR(const Mtx m, const Vec* src, Vec* dst) {
  Vec vTmp;

  MTX_ASSERT(m && "MTXMultVecSR():  NULL MtxPtr 'm' ");
  MTX_ASSERT(src && "MTXMultVecSR():  NULL VecPtr 'src' ");
  MTX_ASSERT(dst && "MTXMultVecSR():  NULL VecPtr 'dst' ");

  vTmp.x = (m[0][2] * src->z) + ((m[0][0] * src->x) + (m[0][1] * src->y));
  vTmp.y = (m[1][2] * src->z) + ((m[1][0] * src->x) + (m[1][1] * src->y));
  vTmp.z = (m[2][2] * src->z) + ((m[2][0] * src->x) + (m[2][1] * src->y));
  dst->x = vTmp.x;
  dst->y = vTmp.y;
  dst->z = vTmp.z;
}

void C_MTXMultVecArraySR(const Mtx m, const Vec* srcBase, Vec* dstBase, u32 count) {
  u32 i;
  Vec vTmp;

  MTX_ASSERT(m && "MTXMultVecArraySR():  NULL MtxPtr 'm' ");
  MTX_ASSERT(srcBase && "MTXMultVecArraySR():  NULL VecPtr 'srcBase' ");
  MTX_ASSERT(dstBase && "MTXMultVecArraySR():  NULL VecPtr 'dstBase' ");
  MTX_ASSERT(count > 1 && "MTXMultVecArraySR():  count must be greater than 1.");

  for(i = 0; i < count; i++) {
    vTmp.x = (m[0][2] * srcBase->z) + ((m[0][0] * srcBase->x) + (m[0][1] * srcBase->y));
    vTmp.y = (m[1][2] * srcBase->z) + ((m[1][0] * srcBase->x) + (m[1][1] * srcBase->y));
    vTmp.z = (m[2][2] * srcBase->z) + ((m[2][0] * srcBase->x) + (m[2][1] * srcBase->y));
    dstBase->x = vTmp.x;
    dstBase->y = vTmp.y;
    dstBase->z = vTmp.z;
    srcBase++;
    dstBase++;
  }
}

void C_MTXROMultVecArray(const ROMtx m, const Vec *srcBase, Vec *dstBase, u32 count)
{
  u32 i;
  Vec vTmp;

  MTX_ASSERT(m && "MTXROMultVecArray():  NULL MtxPtr 'm' ");
  MTX_ASSERT(srcBase && "MTXROMultVecArray():  NULL VecPtr 'srcBase' ");
  MTX_ASSERT(dstBase && "MTXROMultVecArray():  NULL VecPtr 'dstBase' ");

  for(i = 0; i < count; i++) {
    vTmp.x = (m[0][0] * srcBase->x) + (m[1][0] * srcBase->y) + (m[2][0] * srcBase->z) + m[3][0];
    vTmp.y = (m[0][1] * srcBase->x) + (m[1][1] * srcBase->y) + (m[2][1] * srcBase->z) + m[3][1];
    vTmp.z = (m[0][2] * srcBase->x) + (m[1][2] * srcBase->y) + (m[2][2] * srcBase->z) + m[3][2];
    dstBase->x = vTmp.x;
    dstBase->y = vTmp.y;
    dstBase->z = vTmp.z;
    srcBase++;
    dstBase++;
  }
}
