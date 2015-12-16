# CG-asst5

## Ningna Wang(ningnaw)

#### Bidirectional Path Tracing

I’m expecting this algorithm can handle indirect lighting problems far more efficiently and robustly than ordinary path tracing.

###### 1. Alogorithm:
- Suppose we go from camera to create the ray path: p1->p2->...->pi, so this time we also create a ray path from light: q1->q2->...->qj. Therefore, the total ray path should be:

>>>>>>>>>>>>>>> p = p1->p2->...->pi->qj->....q2->q1

- Trace a shadow ray from pi to qj to see if they are visible
- If so, that means above path carries energy from the light to camera. Then we can evaluate
this path’s contribution accordingly.
- If not, which means the corresponding pixel in camera should be in shadow


###### 2. How to build:
-  mkdir build && cd build && cmake .. && make
-  ./pathtracer -m 4 -s 8 ../dae/sky/CBspheres_lambertian.dae

###### 3. Attention:
-  only support area light

###### 4. Results:



