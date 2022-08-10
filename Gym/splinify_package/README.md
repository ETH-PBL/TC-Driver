A small package to handle a splinified racetrack 

The most common way to use it is importing it 
```
from splinify.splinify import SplineTrack
```
and then instantiating a track object

```
track = SplineTrack(\path\to\track\waypoints, \path\to\optimal\trajectory\waypoints, safety_margin)
```
Both the optimal trajectory and the safety margin are optional. Default valueas are the center line and 0 respectively.

NOTE: 
when the trajectory is given, a certain constant difference might be present with respect to the parameter of the track. I.e. `track.trajectory.get_coordinate(theta_hat)` IS NOT NECESSARILY the nearest point to `track.get_coordinate(theta_hat)`. To remove the delta a quick (but imprecise) way is `track.trajectory.get_coordinate(theta_hat+track.delta_traj)`. This point will still be quite imprecise as the track advances with a different pace of the trajectory (one metre on the track can be not aligned with one metre on the trajectory). The suggested usage, for having a better point is then:

```
point_on_track = track.get_coordinate(theta_hat)
theta_trajectory = track.trajectory.find_theta(point_on_track, theta_hat+track.delta_traj)
point_on_trajectory = track.trajectory.get_coordinate(theta_trajectory)
```
This however comes with an additional computational burden. 
(On my machine, with an i5-6200U, circa 9 additional ms)
