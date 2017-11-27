# Bidirectional Path Tracing

CMU 15-462/662 personal project in 2015Fall (http://15462.courses.cs.cmu.edu/fall2015/)

## Ningna Wang(ningnaw)

#### Bidirectional Path Tracing

I’m expecting this algorithm can handle indirect lighting problems far more efficiently and robustly than ordinary path tracing.

##### 1. Alogorithm:
- Suppose we go from camera to create the ray path: p1->p2->...->pi, so this time we also create a ray path from light: q1->q2->...->qj. Therefore, the total ray path should be: </br>


<p align="center">
  p = p1->p2->...->pi->qj->....q2->q1
</p>

- Trace a shadow ray from pi to qj to see if they are visible
- If so, that means above path carries energy from the light to camera. Then we can evaluate
this path’s contribution accordingly.
- If not, which means the corresponding pixel in camera should be in shadow


##### 2. How to build:
-  mkdir build && cd build && cmake .. && make
-  ./pathtracer -m 5 -s 8 -t 10../dae/sky/CBspheres_lambertian.dae

##### 3. Attention:
-  <strong> only support area light </strong> for now
- -m: bounce time </br>
- -s: sample pixel </br>
- -t: threads </br>

##### 4. Results:

###### Example 1 (lambertian + 128 pixels):
- Bidirectional Path Tracing:

$ ./pathtracer -m 5 -s 128 -t 100../dae/sky/CBspheres_lambertian.dae

![alt tag](https://github.com/junanita/CG-asst5/blob/master/asst5/result/128_lam_B.png)

</br>
- Classis Path tracing:

$ ./pathtracer -m 5 -s 128 -t 100../dae/sky/CBspheres_lambertian.dae

![alt tag](https://github.com/junanita/CG-asst5/blob/master/asst5/result/128_lam_ref.png)
</br></br></br>


###### Example 2 (lambertian + 512 pixels):
- Bidirectional Path Tracing:

$ ./pathtracer -m 5 -s 512 -t 100../dae/sky/CBspheres_lambertian.dae

![alt tag](https://github.com/junanita/CG-asst5/blob/master/asst5/result/512_lam_B.png)

</br>
- Classis Path tracing:

$ ./pathtracer -m 5 -s 512 -t 100../dae/sky/CBspheres_lambertian.dae

![alt tag](https://github.com/junanita/CG-asst5/blob/master/asst5/result/512_lam_ref.png)
</br></br></br>

###### Example 3 (128 pixels):
- Bidirectional Path Tracing:

$ ./pathtracer -m 5 -s 128 -t 100../dae/sky/CBspheres.dae

![alt tag](https://github.com/junanita/CG-asst5/blob/master/asst5/result/128_B.png)

</br>
- Classis Path tracing:

$ ./pathtracer -m 5 -s 128 -t 100../dae/sky/CBspheres.dae

![alt tag](https://github.com/junanita/CG-asst5/blob/master/asst5/result/128_B.png)


</br></br></br>
##### 5. Analysis:
From above reuslt pictures, we can see that examples rendering using Bidirectional Path Tracing(BPT) showed very good result at 128 to 256 samples per pixel comparing to the reference results. However, the rendering time of BPT is slightly longer than classic path tracing.
