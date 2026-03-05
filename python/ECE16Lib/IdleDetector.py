from ECE16Lib.Communication import Communication
from ECE16Lib.CircularList import CircularList
from matplotlib import pyplot as plt
from time import time

class IdleDetector:
    inactive_time=10
    message_active=""
    message_inactive=""
    __serial_name = ""
    __baud_rate = 115200
    def __init__(self,time,serial_name,baud_rate):
        self.inactive_time=time
        #self.message_active=active
        #self.message_inactive_5=inactive_5
        #self.message_inactive_10=inactive_10
        self.__serial_name=serial_name
        self.__baud_rate=baud_rate
        self.__previous_time_plot=0
        self.state_gen=0
        
    def IdleDetection(self,comms):
        num_samples = 250               # 5 seconds of data @ 50Hz           # update the plot every 0.05s (20 FPS)
        ax = CircularList([], num_samples)
        ay = CircularList([], num_samples)
        az = CircularList([], num_samples)
        L1 = CircularList([], num_samples)
        comms=comms
        state=1
        #0:  OFF state
        #1: ON/Active state
        #2: inactive / LED blink 
        buzzer=0
        message=None

        #comms = Communication("COM11", 115200)
        #comms.clear()                   # just in case any junk is in the pipes
        #comms.send_message("w(earable")  # begin sending data

        
        previous_time = time()
        previous_time_read = time()

        while(self.state_gen==0):
            current_time = time()
            if (current_time-previous_time_read>=0.02):
              message = comms.receive_message()

            
            
        # if (str(message).strip() == "start"):
        #   state=1
        #   continue
        # elif (str(message).strip() == "stop"):
        #   state=0
        #   buzzer=0
          #continue
            if(message != None):
              #print(message)
                  
              if(","in message) :
                  (m1, m2, m3) = message.split(',')
                  ax.add(int(m1))
                  ay.add(int(m2))
                  az.add(int(m3))
                  L1.add(abs(ax[-1])+abs(ay[-1])+abs(az[-1])) 
            
   
                  if (state==0):
                    previous_time=time()  
                  if (state==1):
                   if (abs(L1[-1]-6000)>800):
                       previous_time=time()
                       comms.send_message("active")
                       print("active")
                   elif (current_time-previous_time>5):
                      print("inac5")
                      comms.send_message("inactive_5")
                      state=2
                  if(state==2):
                   if (abs(L1[-1]-6000)>800):
                         previous_time=time()
                         print("active")
                         comms.send_message("active")
                         state=1
                         buzzer=0
                  if(current_time-previous_time>10 and buzzer==0): #if the motor was never activated and user innactive for >10s
                      print("inac10")
                      comms.send_message("inactive_10") 
                      buzzer=1 #allows us to buzz the motor only once
              else:
                  self.state_gen=int(message)
                  
            #time.sleep(0.02)))))
                    

        
    def __Plotting(self, ax, ay, az, refresh_time, current_time):
        if (current_time - self.__previous_time_plot > refresh_time):
          self.__previous_time_plot = current_time
          plt.subplot(3,1,1)
          plt.cla()
          plt.title("ax")
          plt.plot(ax)
          plt.subplot(3,1,2)
          plt.cla()
          plt.title("ay")
          plt.plot(ay)
          plt.subplot(3,1,3)
          plt.cla()
          plt.title("ay")
          plt.plot(az)
          plt.pause(0.001)
        
        
        
        
    