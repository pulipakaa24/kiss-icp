Adaptation of https://github.com/PRBonn/kiss-icp

Allows for usage like the following:

```KissICP(KISSConfig)``` (Original) or ```KissICP(KISSConfig, String Map_Path)```

Where ```Map_Path``` denotes the path to a global map used instead of the locally generated one.

My use case was to use KISS_Slam, created by the same lab as KISS_ICP, to create a map with numerous map closures, then use that robust map to have more accurate lidar odometry.
