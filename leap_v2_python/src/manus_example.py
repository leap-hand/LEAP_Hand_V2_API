from software.leap_v2_python.src.leap_v2_node import LeapNode
import numpy as np
import zmq
import threading
import time

class ZMQSubscriber:
    """
    Creates a thread that subscribes to a ZMQ publisher
    """
    def __init__(self):
        context = zmq.Context()
        self.socket = context.socket(zmq.PULL)
        #self.socket.setsockopt(zmq.CONFLATE, True)     
        self.socket.connect("tcp://localhost:8000")
        self._subscriber_thread = threading.Thread(target=self._update_value)
        self._subscriber_thread.start()
        self._value = None
        self.last_message = None
    @property
    def message(self):
        return self._value
    #This thread runs in the background and receives the messages
    def _update_value(self):
        while True:
            message = self.socket.recv()
            message = message.decode('utf-8')
            message = message.split(",") 
            if len(message) == 40:
                self._value = list(map(float,message[20:40]))  #Get the right hand data (second half of the 40 datapoints coming in)

if __name__ == "__main__":
    zmq_sub = ZMQSubscriber()
    leap_node = LeapNode()
    try:
        x = time.time()
        while True:
            if zmq_sub.message is None:
                print("No data from gloves")
            else:
                pose = zmq_sub.message        
                # Data in the finger order of index, middle, ring, thumb, etc.
                data = np.deg2rad(pose[4:16] + pose[0:4]) + np.array([0.0,0.0,0.0,0.0,  0.0,0.0,0,0,  0.0,0.0,0,0,  0,0,0.2,0.2])  
                output = np.zeros(8)
                side_mtp = -1.5
                forward_mtp = 1.5
                output[[0,2,4,6]] = data[[0,4,8,12]] * side_mtp
                output[6] = output[6] * 1.0 + 0.5 #thumb adjust
                output[1] = np.mean(data[1:4])*forward_mtp - 0.1
                output[3] = np.mean(data[5:8])*forward_mtp - 0.1
                output[5] = np.mean(data[9:12])*forward_mtp - 0.1
                output[7] = np.mean(data[13:16])*forward_mtp - 0.1
                leap_node.write_leap(output)
                print(output)
                print(leap_node.read_eff())
            print(time.time() - x)
            x = time.time()
            time.sleep(0.0025)
    except KeyboardInterrupt:
        print("Interrupted by user.")
        leap_node.close()
    finally:
        leap_node.close()
