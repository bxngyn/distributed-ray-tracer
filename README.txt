LINK: https://youtu.be/lzIVSQKz8-w

EFFECTS:
    1. Soft Shadows: 
        - Can be seen in frame 0 and 250 from the stickman's shadow
        - Implemented by creating AreaLight class, which is derived from Light base class. In rayColor, I generate 48 random samples on the area light for every pixel, and shoot rays out to each of the points to calculate lighting contributions. Then, I divide by 48 to get the average lighting contribution, resulting in the soft shadow effect
    2. Glossy Reflections:
        - Can be seen in the large sphere levitating above stickman
        - Implemented by generating 48 rays that "jitter" around the reflection ray using random angles from the reflection ray direction, and then sampling the average color contribution of all the rays

BUILD INSTRUCTIONS:
    To compile, make sure you are in the Nguyen_Brandon_Final folder with the Eigen folder also inside. Run make, which will create an executable called previz. Run ./previz. The output will be frame.0250.ppm, a representative frame of my project. (takes ~35 seconds on my local machine and ~20 seconds on the Zoo)
    
    **note: I commented out the renderImage function that uses anti-aliasing so you can generate the frame faster, but when rendering my full video I had the other one uncommented.
