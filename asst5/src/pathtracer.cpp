#include "pathtracer.h"
#include "bsdf.h"
#include "ray.h"

#include <stack>
#include <random>
#include <algorithm>

#include "CMU462/CMU462.h"
#include "CMU462/vector3D.h"
#include "CMU462/matrix3x3.h"
#include "CMU462/lodepng.h"

#include "GL/glew.h"

#include "random_util.h"

#include "static_scene/sphere.h"
#include "static_scene/triangle.h"
#include "static_scene/light.h"

#include <cassert>

using namespace CMU462::StaticScene;

using std::min;
using std::max;

namespace CMU462 {

// #define ENABLE_RAY_LOGGING 1

PathTracer::PathTracer(size_t ns_aa,
                       size_t max_ray_depth, size_t ns_area_light,
                       size_t ns_diff, size_t ns_glsy, size_t ns_refr,
                       size_t num_threads, HDRImageBuffer* envmap) {
  state = INIT,
  this->ns_aa = ns_aa;
  set_sample_pattern();
  this->max_ray_depth = max_ray_depth;
  this->ns_area_light = ns_area_light;
  this->ns_diff = ns_diff;
  this->ns_glsy = ns_diff;
  this->ns_refr = ns_refr;

  // if (envmap) {
  //   this->envLight = new EnvironmentLight(envmap);
  // } else {
    this->envLight = NULL;
  // }

  bvh = NULL;
  scene = NULL;
  camera = NULL;

  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  show_rays = true;

  imageTileSize = 32;
  numWorkerThreads = num_threads;
  workerThreads.resize(numWorkerThreads);

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;

}

PathTracer::~PathTracer() {

  delete bvh;
  delete gridSampler;
  delete hemisphereSampler;

}

void PathTracer::set_scene(Scene *scene) {

  if (state != INIT) {
    return;
  }

  if (this->scene != nullptr) {
    delete scene;
    delete bvh;
    selectionHistory.pop();
  }

  if (this->envLight != nullptr) {
    scene->lights.push_back(this->envLight);
  }

  this->scene = scene;
  build_accel();

  if (has_valid_configuration()) {
    state = READY;
  }
}

void PathTracer::set_camera(Camera *camera) {

  if (state != INIT) {
    return;
  }

  this->camera = camera;
  if (has_valid_configuration()) {
    state = READY;
  }

}

void PathTracer::set_frame_size(size_t width, size_t height) {
  if (state != INIT && state != READY) {
    stop();
  }
  sampleBuffer.resize(width, height);
  frameBuffer.resize(width, height);
  if (has_valid_configuration()) {
    state = READY;
  }
}

bool PathTracer::has_valid_configuration() {
  return scene && camera && gridSampler && hemisphereSampler &&
         (!sampleBuffer.is_empty());
}

void PathTracer::update_screen() {
  switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      visualize_accel();
      break;
    case RENDERING:
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      break;
    case DONE:
        //sampleBuffer.tonemap(frameBuffer, tm_gamma, tm_level, tm_key, tm_wht);
      glDrawPixels(frameBuffer.w, frameBuffer.h, GL_RGBA,
                   GL_UNSIGNED_BYTE, &frameBuffer.data[0]);
      break;
  }
}

void PathTracer::stop() {
  switch (state) {
    case INIT:
    case READY:
      break;
    case VISUALIZE:
      while (selectionHistory.size() > 1) {
        selectionHistory.pop();
      }
      state = READY;
      break;
    case RENDERING:
      continueRaytracing = false;
    case DONE:
      for (int i=0; i<numWorkerThreads; i++) {
            workerThreads[i]->join();
            delete workerThreads[i];
        }
      state = READY;
      break;
  }
}

void PathTracer::clear() {
  if (state != READY) return;
  delete bvh;
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  selectionHistory.pop();
  sampleBuffer.resize(0, 0);
  frameBuffer.resize(0, 0);
  state = INIT;
}

void PathTracer::start_visualizing() {
  if (state != READY) {
    return;
  }
  state = VISUALIZE;
}

void PathTracer::start_raytracing() {
  if (state != READY) return;

  rayLog.clear();
  workQueue.clear();

  state = RENDERING;
  continueRaytracing = true;
  workerDoneCount = 0;

  sampleBuffer.clear();
  frameBuffer.clear();
  num_tiles_w = sampleBuffer.w / imageTileSize + 1;
  num_tiles_h = sampleBuffer.h / imageTileSize + 1;
  tile_samples.resize(num_tiles_w * num_tiles_h);
  memset(&tile_samples[0], 0, num_tiles_w * num_tiles_h * sizeof(int));

  // populate the tile work queue
  for (size_t y = 0; y < sampleBuffer.h; y += imageTileSize) {
      for (size_t x = 0; x < sampleBuffer.w; x += imageTileSize) {
          workQueue.put_work(WorkItem(x, y, imageTileSize, imageTileSize));
      }
  }

  // launch threads
  fprintf(stdout, "[PathTracer] Rendering... "); fflush(stdout);
  for (int i=0; i<numWorkerThreads; i++) {
      workerThreads[i] = new std::thread(&PathTracer::worker_thread, this);
  }
}


void PathTracer::build_accel() {

  // collect primitives //
  fprintf(stdout, "[PathTracer] Collecting primitives... "); fflush(stdout);
  timer.start();
  vector<Primitive *> primitives;
  for (SceneObject *obj : scene->objects) {
    const vector<Primitive *> &obj_prims = obj->get_primitives();
    primitives.reserve(primitives.size() + obj_prims.size());
    primitives.insert(primitives.end(), obj_prims.begin(), obj_prims.end());
  }
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // build BVH //
  fprintf(stdout, "[PathTracer] Building BVH... "); fflush(stdout);
  timer.start();
  bvh = new BVHAccel(primitives);
  timer.stop();
  fprintf(stdout, "Done! (%.4f sec)\n", timer.duration());

  // initial visualization //
  selectionHistory.push(bvh->get_root());
}

void PathTracer::log_ray_miss(const Ray& r) {
    rayLog.push_back(LoggedRay(r, -1.0));
}

void PathTracer::log_ray_hit(const Ray& r, double hit_t) {
    rayLog.push_back(LoggedRay(r, hit_t));
}

void PathTracer::visualize_accel() const {

  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glLineWidth(1);
  glEnable(GL_DEPTH_TEST);

  // hardcoded color settings
  Color cnode = Color(.5, .5, .5, .25);
  Color cnode_hl = Color(1., .25, .0, .6);
  Color cnode_hl_child = Color(1., 1., 1., .6);

  Color cprim_hl_left = Color(.6, .6, 1., 1);
  Color cprim_hl_right = Color(.8, .8, 1., 1);
  Color cprim_hl_edges = Color(0., 0., 0., 0.5);

  BVHNode *selected = selectionHistory.top();

  // render solid geometry (with depth offset)
  glPolygonOffset(1.0, 1.0);
  glEnable(GL_POLYGON_OFFSET_FILL);

  if (selected->isLeaf()) {
    for (size_t i = 0; i < selected->range; ++i) {
       bvh->primitives[selected->start + i]->draw(cprim_hl_left);
    }
  } else {
      if (selected->l) {
          BVHNode* child = selected->l;
          for (size_t i = 0; i < child->range; ++i) {
              bvh->primitives[child->start + i]->draw(cprim_hl_left);
          }
      }
      if (selected->r) {
          BVHNode* child = selected->r;
          for (size_t i = 0; i < child->range; ++i) {
              bvh->primitives[child->start + i]->draw(cprim_hl_right);
          }
      }
  }

  glDisable(GL_POLYGON_OFFSET_FILL);

  // draw geometry outline
  for (size_t i = 0; i < selected->range; ++i) {
      bvh->primitives[selected->start + i]->drawOutline(cprim_hl_edges);
  }

  // keep depth buffer check enabled so that mesh occluded bboxes, but
  // disable depth write so that bboxes don't occlude each other.
  glDepthMask(GL_FALSE);

  // create traversal stack
  stack<BVHNode *> tstack;

  // push initial traversal data
  tstack.push(bvh->get_root());

  // draw all BVH bboxes with non-highlighted color
  while (!tstack.empty()) {

    BVHNode *current = tstack.top();
    tstack.pop();

    current->bb.draw(cnode);
    if (current->l) tstack.push(current->l);
    if (current->r) tstack.push(current->r);
  }

  // draw selected node bbox and primitives
  if (selected->l) selected->l->bb.draw(cnode_hl_child);
  if (selected->r) selected->r->bb.draw(cnode_hl_child);

  glLineWidth(3.f);
  selected->bb.draw(cnode_hl);

  // now perform visualization of the rays
  if (show_rays) {
      glLineWidth(1.f);
      glBegin(GL_LINES);

      for (size_t i=0; i<rayLog.size(); i+=500) {

          const static double VERY_LONG = 10e4;
          double ray_t = VERY_LONG;

          // color rays that are hits yellow
          // and rays this miss all geometry red
          if (rayLog[i].hit_t >= 0.0) {
              ray_t = rayLog[i].hit_t;
              glColor4f(1.f, 1.f, 0.f, 0.1f);
          } else {
              glColor4f(1.f, 0.f, 0.f, 0.1f);
          }

          Vector3D end = rayLog[i].o + ray_t * rayLog[i].d;

          glVertex3f(rayLog[i].o[0], rayLog[i].o[1], rayLog[i].o[2]);
          glVertex3f(end[0], end[1], end[2]);
      }
      glEnd();
  }

  glDepthMask(GL_TRUE);
  glPopAttrib();
}

void PathTracer::key_press(int key) {

  BVHNode *current = selectionHistory.top();
  switch (key) {
  case ']':
      ns_aa *=2;
      set_sample_pattern();
      printf("Samples per pixel changed to %lu\n", ns_aa);
      //tm_key = clamp(tm_key + 0.02f, 0.0f, 1.0f);
      break;
  case '[':
      //tm_key = clamp(tm_key - 0.02f, 0.0f, 1.0f);
      ns_aa /=2;
      if (ns_aa < 1) ns_aa = 1;
      set_sample_pattern();
      printf("Samples per pixel changed to %lu\n", ns_aa);
      break;
  case KEYBOARD_UP:
      if (current != bvh->get_root()) {
          selectionHistory.pop();
      }
      break;
  case KEYBOARD_LEFT:
      if (current->l) {
          selectionHistory.push(current->l);
      }
      break;
  case KEYBOARD_RIGHT:
      if (current->l) {
          selectionHistory.push(current->r);
      }
      break;
  case 'a':
  case 'A':
      show_rays = !show_rays;
  default:
      return;
  }
}

// If ns_aa == 1, sample from the middle of the pixel. Otherwise, decompose your
// number ns_aa into a sum of perfect squares, and do stratified sampling on
// each of the respective grids.
void PathTracer::set_sample_pattern() {
  sample_grids.clear();
  if (ns_aa == 1) return;
  int nSamples = ns_aa;
  while (nSamples > 0) {
    int root = (int) sqrt(nSamples);
    int rsq = root * root;
    while (nSamples >= rsq) {
      sample_grids.push_back(root);
      nSamples -= rsq;
    }
  }
}


void PathTracer::generate_path(Ray &r, std::vector<StaticScene::VertexInfo> &vertices, bool isEye) {
  
  Intersection isect;
  Spectrum cumulative = Spectrum(1.0f, 1.0f, 1.0f); //empty 

  int init_depth = r.depth;

  while (true) {

    if (!bvh->intersect(r, &isect)) { //interact the ray with scene
      // log ray miss
      #ifdef ENABLE_RAY_LOGGING
      log_ray_miss(r);
      #endif

      break;
    }

    // log ray hit
    #ifdef ENABLE_RAY_LOGGING
    log_ray_hit(r, isect.t);
    #endif

    // create vertex info
    VertexInfo v;
    const Vector3D& hit_p = r.o + r.d * isect.t;
    v.p = hit_p;
    v.n = isect.n;
    v.bsdf = isect.bsdf;


    // make a coordinate system for a hit point
    // with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o(o2w.T());

    if (isEye) { // eye/camera path

      // w_out points towards the source of the ray (e.g.,
      // toward the camera if this is a primary ray)
      const Vector3D& w_out = (w2o * (r.o - hit_p)).unit();
      v.wo = w_out;

      if (r.depth == 0) break;

      //  hit a emissive material eg. light
      //  this condition should be handle ahead 
      //  otherwise it will be terminated during Russian Roulette
      if (v.bsdf->get_emission() != Spectrum()) { // hit the light/glass
        v.cumulative = cumulative;
        vertices.push_back(v);
        break;
      }

      float pdf; Vector3D w_in;
      const Spectrum& f = v.bsdf->sample_f(w_out, &w_in, &pdf);

      // generate new ray
      // compute reflected / refracted ray direction
      const Vector3D& w_in_world = (o2w * w_in).unit();
      v.wi = w_in_world;
      Ray rec(hit_p + EPS_D * w_in_world, w_in_world, INF_D, r.depth - 1);


      // Russian Roulette
      //
      // Pick a uniform random value p, and terminate path if p < terminate_prob.
      // Otherwise, trace the ray in the direction of w_in
      float reflectance = clamp(0.f, 1.f, f.illum());
      float terminate_prob = 1.f - reflectance;

      if (coin_flip(terminate_prob)) break;
      
      // compute Monte Carlo estimator by tracing ray (result weighted
      // by (1-terminate_prob) to account for Russian roulette)
      double cos_theta = fabs(w_in.z);
      double denom = pdf * (1 - terminate_prob);
      double scale = denom ? cos_theta / denom : 0;

      // update vertex info
      cumulative *= scale * f;
      r = rec;

    }
    else { // light path

      // w_in points towards the source of the ray (e.g.,
      // toward the light if this is a primary ray)
      const Vector3D& w_in = (w2o * (r.o - hit_p)).unit(); // TODO
      v.wi = w_in;

      if (r.depth == 0) break;

      float pdf;
      Vector3D w_out;
      const Spectrum& f = isect.bsdf->sample_f(w_in, &w_out, &pdf);

      // generate new ray
      // compute reflected / refracted ray direction
      const Vector3D& w_out_world = (o2w * w_out).unit(); // TODO
      v.wo = w_out_world;
      Ray rec(hit_p + EPS_D * w_out_world, w_out_world, INF_D, r.depth - 1);

      // Russian Roulette
      //
      // Pick a uniform random value p, and terminate path if p < terminate_prob.
      // Otherwise, trace the ray in the direction of w_in
      float reflectance = clamp(0.f, 1.f, f.illum());
      float terminate_prob = 1.f - reflectance;

      if (coin_flip(terminate_prob)) break;

      // compute Monte Carlo estimator by tracing ray (result weighted
      // by (1-terminate_prob) to account for Russian roulette)
      double cos_theta = fabs(w_out.z); // TODO
      double denom = pdf * (1 - terminate_prob);
      double scale = denom ? cos_theta / denom : 0;

      // update vertex info
      cumulative *= scale * f;
      r = rec;
    }

    v.cumulative = cumulative;
    vertices.push_back(v);

  }// while end

  return;
}


Spectrum PathTracer::evaluatePath(std::vector<StaticScene::VertexInfo> &eyePath, 
                                  std::vector<StaticScene::VertexInfo> &lightPath, 
                                  int nEye, int nLight) {
  /*  nEye = s, nLight = t
      path: y0...yt,xs...x0
      total s + t + 2 points
      total s + t + 1 edges
      satiesfy max_Length = s + t + 1
  */

  VertexInfo &ev = eyePath[nEye - 1];
  VertexInfo &lv = lightPath[nLight - 1];

  Spectrum L = Spectrum(1.0f, 1.0f, 1.0f);

  // account for reflections on the eye and light paths
  if (nEye > 1)
    L *= eyePath[nEye - 2].cumulative;
  if (nLight > 1)
    L *= lightPath[nLight - 2].cumulative;

  // Calculate the geometric term
  Vector3D etl = lv.p - ev.p;
  double lengthSquared = etl.norm2();
  etl.normalize();

  float geoTerm = abs(dot(etl, ev.n)) * abs(dot(-etl, lv.n)) / lengthSquared;

  // extremely close points cause numerical problems
  if (lengthSquared < 0.05f) {
    return Spectrum();
  }

  // Evaluate BSDFs at last light and eye path surface interaction points (y_s, x_t)
  L *= ev.bsdf->f(ev.wo, etl);
  L *= lv.bsdf->f(lv.wi, -etl);

  L *= geoTerm; 

  return L;
}


Spectrum PathTracer::renderPaths(Ray &eyeRay, Ray &ligthRay, Spectrum &Le, bool includeLe) {

  Vector3D wi;
  Vector3D onLight;
  std::vector<VertexInfo> eyePath;
  std::vector<VertexInfo> lightPath;
  Spectrum L_out = Spectrum();

  generate_path(eyeRay, eyePath, true);
  generate_path(ligthRay, lightPath, false);

  /**
    path y0...yt,xs...x0, from light to eye
    Case ii).   s > 0, t = 0: classic path tracing
    Case iii).  s = 0, t > 0: classic light tracing
    Case iv).   s > 0, t > 0: bidirectioanl path tracing
  **/

  /* Implement case ii) and iv) */
  for (int s = 1; s < eyePath.size() + 1; s++) {
    const VertexInfo &ev = eyePath[s - 1];
    L_out += includeLe ? ev.bsdf->get_emission() : Spectrum();


    // make a coordinate system for a hit point
    // with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, ev.n);
    Matrix3x3 w2o(o2w.T());

    // sample light source from current surface location
    if (!ev.bsdf->is_delta()) {
      Vector3D dir_to_light;
      float dist_to_light;
      float pr;

      //
      // estimate direct lighting integral
      //
      for (SceneLight* light : scene->lights) {

        // no need to take multiple samples from a point/directional source
        int num_light_samples = light->is_delta_light() ? 1 : ns_area_light;

        // integrate light over the hemisphere about the normal
        for (int i = 0; i < num_light_samples; i++) {

          // returns a vector 'dir_to_light' that is a direction from
          // point hit_p to the point on the light source.  It also returns
          // the distance from point x to this point on the light source.
          // (pr is the probability of randomly selecting the random
          // sample point on the light source -- more on this in part 2)

          Spectrum localLe = light->sample_L(ev.p, &dir_to_light, &dist_to_light, &pr);
          // Spectrum localLe = scene->lights[0]->sample_L(ev.p, &dir_to_light, &dist_to_light, &pr);

          // convert direction into coordinate space of the surface, where
          // the surface normal is [0 0 1]
          const Vector3D& w_in = w2o * dir_to_light;
          if (w_in.z < 0) continue;

          // do shadow ray test
          if (!bvh->intersect(Ray(ev.p + EPS_D * ev.n, dir_to_light, dist_to_light))) {
            if (s > 1) {
              localLe *= eyePath[s - 2].cumulative;
            }

            // note that computing dot(n,w_in) is simple
            // in surface coordinates since the normal is (0,0,1)
            double cos_theta = w_in.z;

            // evaluate surface bsdf
            const Spectrum& f = ev.bsdf->f(ev.wo, w_in);

            // L_out += localLe * f * (cos_theta / (num_light_samples * pr));
            // L_out += localLe * f * (cos_theta / (num_light_samples * (s + 1)));
            L_out += localLe * f * (cos_theta / (s + 1));
          }
        }
      }
    }

    /* Case iv).  s > 0, t > 0: bidirectioanl path tracing */ 
    // should satisfy s + t + 1 <= max_ray_depth
    for (int t = 1; t < lightPath.size() + 1; t++) {
      if (t > max_ray_depth - s - 1) break;

      const VertexInfo &lv = lightPath[t - 1];

      Vector3D inner_ray_dir = ev.p + EPS_D * ev.n - lv.p; // TODO
      inner_ray_dir.normalize();

      // do shadow ray test
      if (!bvh->intersect(Ray(ev.p + EPS_D * ev.n, inner_ray_dir))) { // TODO
         L_out += Le * evaluatePath(eyePath, lightPath, s, t) 
                    * (1.0 / (s + t + 1));
      }
    } // iv) end

  } // for end ii) + iv)


  /* Implement case iii) */
  for (int t = 1; t < lightPath.size() + 1; t++) {
    const VertexInfo &lv = lightPath[t - 1];

    Vector3D dir_to_camera = eyeRay.o - lv.p; // TODO
    double lengthSquared = dir_to_camera.norm2();
    dir_to_camera.normalize();

    // do shadow ray test / visibility test
    if (!bvh->intersect(Ray(lv.p, dir_to_camera))) {
      Spectrum localLe = Le;

      if (lengthSquared < 0.05) continue;

      // make a coordinate system for a hit point
      // with N aligned with the Z direction.
      Matrix3x3 o2w;
      make_coord_space(o2w, lv.n);
      Matrix3x3 w2o(o2w.T());

      // w_out points towards the source of the ray (e.g.,
      // toward the camera if this is a primary ray)
      Vector3D w_out = (w2o * (eyeRay.o - lv.p)).unit();
      // note that computing dot(n,w_out) is simple
      // in surface coordinates since the normal is (0,0,1)
      double cos_theta = abs(w_out.z);

      if (t > 1) {
        localLe *= lightPath[t - 2].cumulative;
      }

      // localLe *= lv.bsdf->f(lv.wi, w_out) * cos_theta * (1.0f / lengthSquared); //TODO
      localLe *= lv.bsdf->f(lv.wi, w_out) * cos_theta * (1.0f / lengthSquared); //TODO

      L_out += localLe * (1.0 / (t + 1));
    }
  } // for end case iii).
  
  return L_out;  
}


Spectrum PathTracer::raytrace_pixel(size_t x, size_t y) {

  size_t screenW = sampleBuffer.w;
  size_t screenH = sampleBuffer.h;
  Spectrum L_out = Spectrum();

  if (sample_grids.empty()) {

    Ray eyeRay = camera->generate_ray((x + 0.5) / screenW,
                                      (y + 0.5) / screenH);
    eyeRay.depth = max_ray_depth;
    Ray lightRay = Ray(Vector3D(), Vector3D());
    lightRay.depth = max_ray_depth;
    float lightPdf;

    Spectrum L_temp = Spectrum();
    for (SceneLight* light : scene->lights) {

      // no need to take multiple samples from a point/directional source
      int num_light_samples = light->is_delta_light() ? 1 : ns_area_light;

      // integrate light over the hemisphere about the normal
      for (int i = 0; i < num_light_samples; i++) {
          Spectrum light_L = light->sample_light_ray(&lightRay, &lightPdf);
          light_L = light_L * (1.0 / lightPdf);
          L_temp += renderPaths(eyeRay, lightRay, light_L, false);
      }
      L_out = L_temp * (1.0 / num_light_samples);

    }
    return L_out;
  }


  for (int gridSize : sample_grids) {
    double cellSize = 1.0 / gridSize;
    for (int subY = 0; subY < gridSize; subY++) {
      for (int subX = 0; subX < gridSize; subX++) {

        const Vector2D &p = gridSampler->get_sample();
        double dx = (subX + p.x) * cellSize;
        double dy = (subY + p.y) * cellSize;
        Ray eyeRay = camera->generate_ray((x + dx) / screenW, (y + dy) / screenH);
        eyeRay.depth = max_ray_depth + 1;
        Ray lightRay = Ray(Vector3D(), Vector3D());
        lightRay.depth = max_ray_depth + 1;
        float lightPdf;

        Spectrum L_temp = Spectrum();
        for (SceneLight* light : scene->lights) {

          // no need to take multiple samples from a point/directional source
          int num_light_samples = light->is_delta_light() ? 1 : ns_area_light;

          // integrate light over the hemisphere about the normal
          for (int i = 0; i < num_light_samples; i++) {
              Spectrum light_L = light->sample_light_ray(&lightRay, &lightPdf);
              light_L = light_L * (1.0 / lightPdf);
              L_temp += renderPaths(eyeRay, lightRay, light_L, true);
          }
          L_out += L_temp * (1.0 / num_light_samples);
        }

      }
    }
  }

  return L_out * (1.0 / ns_aa);
}


void PathTracer::raytrace_tile(int tile_x, int tile_y,
                               int tile_w, int tile_h) {

  size_t w = sampleBuffer.w;
  size_t h = sampleBuffer.h;

  size_t tile_start_x = tile_x;
  size_t tile_start_y = tile_y;

  size_t tile_end_x = std::min(tile_start_x + tile_w, w);
  size_t tile_end_y = std::min(tile_start_y + tile_h, h);

  size_t tile_idx_x = tile_x / imageTileSize;
  size_t tile_idx_y = tile_y / imageTileSize;
  size_t num_samples_tile = tile_samples[tile_idx_x + tile_idx_y * num_tiles_w];

  for (size_t y = tile_start_y; y < tile_end_y; y++) {
    if (!continueRaytracing) return;
    for (size_t x = tile_start_x; x < tile_end_x; x++) {
        Spectrum s = raytrace_pixel(x, y);
        sampleBuffer.update_pixel(s, x, y);
    }
  }

  tile_samples[tile_idx_x + tile_idx_y * num_tiles_w] += 1;
  sampleBuffer.toColor(frameBuffer, tile_start_x, tile_start_y, tile_end_x, tile_end_y);
}

void PathTracer::worker_thread() {

  Timer timer;
  timer.start();

  WorkItem work;
  while (continueRaytracing && workQueue.try_get_work(&work)) {
    raytrace_tile(work.tile_x, work.tile_y, work.tile_w, work.tile_h);
  }

  workerDoneCount++;
  if (!continueRaytracing && workerDoneCount == numWorkerThreads) {
    timer.stop();
    fprintf(stdout, "Canceled!\n");
    state = READY;
  }

  if (continueRaytracing && workerDoneCount == numWorkerThreads) {
    timer.stop();
    fprintf(stdout, "Done! (%.4fs)\n", timer.duration());
    state = DONE;
  }
}

void PathTracer::increase_area_light_sample_count() {
  ns_area_light *= 2;
  fprintf(stdout, "[PathTracer] Area light sample count increased to %zu!\n", ns_area_light);
}

void PathTracer::decrease_area_light_sample_count() {
  if (ns_area_light > 1) ns_area_light /= 2;
  fprintf(stdout, "[PathTracer] Area light sample count decreased to %zu!\n", ns_area_light);
}

void PathTracer::save_image() {

  if (state != DONE) return;

  time_t rawtime;
  time (&rawtime);

  string filename = "Screen Shot ";
  filename += string(ctime(&rawtime));
  filename.erase(filename.end() - 1);
  filename += string(".png");

  uint32_t* frame = &frameBuffer.data[0];
  size_t w = frameBuffer.w;
  size_t h = frameBuffer.h;
  uint32_t* frame_out = new uint32_t[w * h];
  for(size_t i = 0; i < h; ++i) {
    memcpy(frame_out + i * w, frame + (h - i - 1) * w, 4 * w);
  }

  fprintf(stderr, "[PathTracer] Saving to file: %s... ", filename.c_str());
  lodepng::encode(filename, (unsigned char*) frame_out, w, h);
  fprintf(stderr, "Done!\n");
}

}  // namespace CMU462
