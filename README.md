# Uniciclo
University project

Starting from the control of a unicycle through the landmarks we want to estimate the position of the unicycle with the Kalman filter and compare it with the real position (measured by the lankmarks). There is also the possibility to "turn off" landmarks
The professor's request was:


Each landmark can be turned on/off by pressing a key on the keyboard: for example,
    by pressing key 1, if landmark 1 is on, it will be turned off. Pressing key 1 again turns it on again.
    The same goes for the other landmarks (key 2 turns landmark 2 on/off and so on).
    For simplicity, in order not to have to use keys other than the numeric ones, assume that the maximum number of landmarks
    cannot be greater than 9. Only the turned on landmarks can be used by the robot: the others are as if they did not exist.
    A landmark is only visible when it is lit and falls within a region of visibility which is a circular sector
    of radius rMax and angle betaMax centered in the unicycle and symmetrical with respect to the forward direction
    of the unicycle itself (see the green region in the figure below).
    The robot can only use measurements from visible landmarks. The measure associated with a visible landmark
    it is the angle (called bearing) at which the robot sees this landmark with respect to its orientation
    (and therefore no longer the distance of the landmark from the robot as happened in the KalmanUniciclo.pde sketch).
    This angle must be considered positive when the landmark is on the left of the robot and negative when it is on the right
    (it is 0 if the landmark is exactly in the direction towards which the unicycle is oriented).
    This measurement is characterized by a Gaussian error with zero mean and standard deviation which must be taken as equal to 10 degrees.
    The region of visibility must be modifiable in real time from the keyboard: in particular by pressing the 'r' and 'R' keys
    it must be possible to decrease and, respectively, increase the maximum range rMax of the circular sector.
    Similarly, with the 'b' and 'B' keys, it must be possible to vary the maximum bearing angle betaMax which defines
    the opening of the circular sector (in the sense that the opening angle of this sector is 2*betaMax).
    Provide a minimum positive value for rangeMax (for example 50 pixels) and limit values ​​for betaMax which must always
    be between 0 and 180 degrees (for example 10 < betaMax < 140 degrees). The region of visibility must be drawn.
    The turned off landmarks, those turned on but not visible and those visible must be drawn with a color (or level of gray)
    different to describe its state (so three possible colors or levels of gray in all).
    Leave active the functionality already present in the KalmanUniciclo.pd sketch and modify the tStep time with the arrows
    between one measure and another. As happens in KalmanUniciclo.pde, when no measure is available in step k
    (this occurs not only when a tStep time has not passed since the last measurement but also when there are no visible landmarks),
    the Kalman filter uses only the prediction based on the wheel encoder steps.
