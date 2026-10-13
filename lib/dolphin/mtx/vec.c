#include "mtx_assert.h"
#include <math.h>
#include <dolphin/mtx.h>
#include <dolphin/ppc_math.h>

void C_VECAdd(const Vec* a, const Vec* b, Vec* ab) {
  MTX_ASSERT(a && "VECAdd():  NULL VecPtr 'a' ");
  MTX_ASSERT(b && "VECAdd():  NULL VecPtr 'b' ");
  MTX_ASSERT(ab && "VECAdd():  NULL VecPtr 'ab' ");
  ab->x = a->x + b->x;
  ab->y = a->y + b->y;
  ab->z = a->z + b->z;
}

void C_VECSubtract(const Vec* a, const Vec* b, Vec* a_b) {
  MTX_ASSERT(a && "VECSubtract():  NULL VecPtr 'a' ");
  MTX_ASSERT(b && "VECSubtract():  NULL VecPtr 'b' ");
  MTX_ASSERT(a_b && "VECSubtract():  NULL VecPtr 'a_b' ");
  a_b->x = a->x - b->x;
  a_b->y = a->y - b->y;
  a_b->z = a->z - b->z;
}

void C_VECScale(const Vec* src, Vec* dst, f32 scale) {
  MTX_ASSERT(src && "VECScale():  NULL VecPtr 'src' ");
  MTX_ASSERT(dst && "VECScale():  NULL VecPtr 'dst' ");
  dst->x = (src->x * scale);
  dst->y = (src->y * scale);
  dst->z = (src->z * scale);
}

void C_VECNormalize(const Vec* src, Vec* unit) {
  f32 mag;

  MTX_ASSERT(src && "VECNormalize():  NULL VecPtr 'src' ");
  MTX_ASSERT(unit && "VECNormalize():  NULL VecPtr 'unit' ");

  mag = (src->z * src->z) + ((src->x * src->x) + (src->y * src->y));
  MTX_ASSERT(0.0f != mag && "VECNormalize():  zero magnitude vector ");

  mag = 1.0f/ sqrtf(mag);
  unit->x = src->x * mag;
  unit->y = src->y * mag;
  unit->z = src->z * mag;
}

void PSVECNormalize(const Vec* src, Vec* unit) {
  f32 sqsum;
  f32 rsqrt;

  MTX_ASSERT(src && "VECNormalize():  NULL VecPtr 'src' ");
  MTX_ASSERT(unit && "VECNormalize():  NULL VecPtr 'unit' ");

  sqsum = (src->z * src->z + src->x * src->x) + src->y * src->y;
  MTX_ASSERT(0.0f != sqsum && "VECNormalize():  zero magnitude vector ");

  rsqrt = ppc_rsqrte(sqsum);
  unit->x = src->x * rsqrt;
  unit->y = src->y * rsqrt;
  unit->z = src->z * rsqrt;
}

f32 C_VECSquareMag(const Vec* v) {
  f32 sqmag;

  MTX_ASSERT(v && "VECMag():  NULL VecPtr 'v' ");

  sqmag = v->z * v->z + ((v->x * v->x) + (v->y * v->y));
  return sqmag;
}

f32 PSVECSquareMag(const Vec* v) {
  f32 xx;
  f32 yy;
  f32 xx_zz;

  MTX_ASSERT(v && "VECMag():  NULL VecPtr 'v' ");

  xx = v->x * v->x;
  yy = v->y * v->y;
  xx_zz = fmaf(v->z, v->z, xx);
  return fmaf(xx_zz, 1.0f, yy);
}

f32 C_VECMag(const Vec* v) {
  return sqrtf(C_VECSquareMag(v));
}

f32 PSVECMag(const Vec* v) {
  f32 sqmag;
  sqmag = PSVECSquareMag(v);
  if (sqmag == 0.0f) {
    return 0.0f;
  }
  return sqmag * ppc_rsqrte(sqmag);
}

f32 C_VECDotProduct(const Vec* a, const Vec* b) {
  f32 dot;

  MTX_ASSERT(a && "VECDotProduct():  NULL VecPtr 'a' ");
  MTX_ASSERT(b && "VECDotProduct():  NULL VecPtr 'b' ");
  dot = (a->z * b->z) + ((a->x * b->x) + (a->y * b->y));
  return dot;
}

void C_VECCrossProduct(const Vec* a, const Vec* b, Vec* axb) {
  Vec vTmp;

  MTX_ASSERT(a && "VECCrossProduct():  NULL VecPtr 'a' ");
  MTX_ASSERT(b && "VECCrossProduct():  NULL VecPtr 'b' ");
  MTX_ASSERT(axb && "VECCrossProduct():  NULL VecPtr 'axb' ");

  vTmp.x = (a->y * b->z) - (a->z * b->y);
  vTmp.y = (a->z * b->x) - (a->x * b->z);
  vTmp.z = (a->x * b->y) - (a->y * b->x);
  axb->x = vTmp.x;
  axb->y = vTmp.y;
  axb->z = vTmp.z;
}

void C_VECHalfAngle(const Vec* a, const Vec* b, Vec* half) {
  Vec aTmp;
  Vec bTmp;
  Vec hTmp;

  MTX_ASSERT(a && "VECHalfAngle():  NULL VecPtr 'a' ");
  MTX_ASSERT(b && "VECHalfAngle():  NULL VecPtr 'b' ");
  MTX_ASSERT(half && "VECHalfAngle():  NULL VecPtr 'half' ");

  aTmp.x = -a->x;
  aTmp.y = -a->y;
  aTmp.z = -a->z;
  bTmp.x = -b->x;
  bTmp.y = -b->y;
  bTmp.z = -b->z;

  VECNormalize(&aTmp, &aTmp);
  VECNormalize(&bTmp, &bTmp);
  VECAdd(&aTmp, &bTmp, &hTmp);

  if (VECDotProduct(&hTmp, &hTmp) > 0.0f) {
    VECNormalize(&hTmp, half);
    return;
  }
  *half = hTmp;
}

void C_VECReflect(const Vec* src, const Vec* normal, Vec* dst) {
  f32 cosA;
  Vec uI;
  Vec uN;

  MTX_ASSERT(src && "VECReflect():  NULL VecPtr 'src' ");
  MTX_ASSERT(normal && "VECReflect():  NULL VecPtr 'normal' ");
  MTX_ASSERT(dst && "VECReflect():  NULL VecPtr 'dst' ");

  uI.x = -src->x;
  uI.y = -src->y;
  uI.z = -src->z;

  VECNormalize(&uI, &uI);
  VECNormalize(normal, &uN);

  cosA = VECDotProduct(&uI, &uN);
  dst->x = (2.0f * uN.x * cosA) - uI.x;
  dst->y = (2.0f * uN.y * cosA) - uI.y;
  dst->z = (2.0f * uN.z * cosA) - uI.z;
  VECNormalize(dst, dst);
}

f32 C_VECSquareDistance(const Vec* a, const Vec* b) {
  Vec diff;

  diff.x = a->x - b->x;
  diff.y = a->y - b->y;
  diff.z = a->z - b->z;
  return (diff.z * diff.z) + ((diff.x * diff.x) + (diff.y * diff.y));
}

f32 PSVECSquareDistance(const Vec* a, const Vec* b) {
  f32 dx = a->x - b->x;
  f32 dy = a->y - b->y;
  f32 dz = a->z - b->z;
  return (dx * dx + dy * dy) + dz * dz;
}

f32 C_VECDistance(const Vec* a, const Vec* b) {
  return sqrtf(C_VECSquareDistance(a, b));
}

f32 PSVECDistance(const Vec* a, const Vec* b) {
  f32 sqdist;
  sqdist = PSVECSquareDistance(a, b);
  if (sqdist == 0.0f) {
    return 0.0f;
  }
  return sqdist * ppc_rsqrte(sqdist);
}
