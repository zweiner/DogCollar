from ECE16Lib.Communication import Communication
import numpy as np
comms = Communication("COM11", 115200)
import numpy as np
# Save data to file
def save_data(filename, data):
  np.savetxt(filename, data, delimiter=",")

filename = ".\\gps_data.csv"
lat=[]
long=[]
try: 
    while(True):
        msg = comms.receive_message()
        if(msg != None):
            try:
              (m1,m2) = msg.split(',')
              lat.append(float(m1))
              long.append(float(m2))
              print(m1)
              print(m2)
            except ValueError: # if corrupted data, skip the sample
              continue
except KeyboardInterrupt:
    print("Arrêt avec Ctrl+C")
    comms.close()
    data = np.column_stack([lat, long])
    save_data(filename, data)


       


