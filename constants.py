REACTION_TIME = 0.15 # s   # TAU # 0.1-0.5 for humans
# LONGITUDINAL QUANTITIES
ACCEL_MAX = 5 # M / S^2
BRAKE_MAX = 8 # M / S^2   # CHECK SIGN
BRAKE_MIN = 4 # M / S^2  # CHECK SIGN
JERK_MAX = 10 # M/S^3
# LATERAL QUANTITIES
LAT_DIST_MARGIN = 0.4 # M    # mu
LAT_ACCEL_MAX = 3 # M/S^2
LAT_MIN_BR = 3 # M/S^2


"""
Values taken from
[2] N. Ar´echiga, “Specifying safety of autonomous vehicles in signal temporal logic,” in 2019 IEEE Intelligent Vehicles Symposium (IV), pp. 58–63, 2019.

[3] M. Hekmatnejad, S. Yaghoubi, A. Dokhanchi, H. Amor, A. Shrivastava, L. Karam, and G. Fainekos, “Encoding and monitoring responsibility sensitive safety rules for automated vehicles in signal temporal logic,” in MEMOCODE 2019 - 17th ACM-IEEE International Conference on Formal Methods and Models for System Design, Association for Computing Machinery, Inc, Oct. 2019.

"""