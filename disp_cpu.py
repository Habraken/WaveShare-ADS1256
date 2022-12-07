from vcgencmd import Vcgencmd
import time

vcgm = Vcgencmd()

while True:
    print("Temperature: ",vcgm.measure_temp()," Celsius")
    print("Core freq. : ",vcgm.measure_clock("core")," Hz")
    print("Arm  freq. : ",vcgm.measure_clock("arm")," Hz")
    print("\33[4A")
    time.sleep(2)