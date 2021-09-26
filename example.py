'''
Example Python code for the ahrs.py library. 

:devices: - Rapsberry Pi (3/4)
          - Parker-LORD MicroStrain AHRS IMU 

To run, enter the following into the terminal (exclude the $): 
    $ python example.py
'''

from lib.ahrs import AHRS
import time


def ahrs_loop():
    ### initialize AHRS with appropriate COM port
    ahrs1 = AHRS("/dev/ttyACM0")

    while True:
        ### fetch roll, pitch, yaw data
        ahrs_rpy, ahrs_raw = ahrs1.update(degrees=True)
        
        ### only print one at a time for a legible output
        print(ahrs_rpy, end='\r')
        #print(ahrs_raw, end='\r')

        ### you can use the roll, pitch, yaw, and the raw data as follows:
        # roll, pitch, yaw = (ahrs_rpy.roll, ahrs_rpy.pitch, ahrs_rpy.yaw)
        # ax, ay, az = (ahrs_raw.ax, ahrs_raw.ay, ahrs_raw.az)
        # gx, gy, gz = (ahrs_raw.gx, ahrs_raw.gy, ahrs_raw.gz)

        ### to make the output legible - remove if you need data at a faster rate
        time.sleep(0.002)
        

if __name__=="__main__":
    ahrs_loop()
