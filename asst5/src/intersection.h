#ifndef CMU462_INTERSECT_H
#define CMU462_INTERSECT_H

#include <vector>

#include "CMU462/vector3D.h"
#include "CMU462/spectrum.h"
#include "CMU462/misc.h"

#include "bsdf.h"

namespace CMU462 { namespace StaticScene {

class Primitive;

/**
 * A record of an intersection point which includes the time of intersection
 * and other information needed for shading
 */
struct Intersection {

  Intersection() : t (INF_D), primitive(NULL), bsdf(NULL) { }

  double t;    ///< time of intersection

  const Primitive* primitive;  ///< the primitive intersected

  Vector3D n;  ///< normal at point of intersection

  BSDF* bsdf; ///< BSDF of the surface at point of intersection

};



struct VertexInfo {

  VertexInfo() : p(Vector3D(0,0,0)), bsdf(NULL) { }

  Vector3D p; ///< position at point of intersection

  Vector3D n;  ///< normal at point of intersection

  Vector3D wi, wo; // wi and wo all definde in world coordinates 

  BSDF* bsdf; ///< BSDF of the surface at point of intersection

  Spectrum cumulative;

};

} // namespace StaticScene
} // namespace CMU462

#endif // CMU462_INTERSECT_H
