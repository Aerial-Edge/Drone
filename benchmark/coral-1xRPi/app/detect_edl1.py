import cv2
from picamera2 import Picamera2
import tflite_runtime.interpreter as tflite
import time
import numpy as np
from libcamera import controls
import pandas as pd


fps_array = np.zeros(30 * 30)  # Assuming your fps function will be called 60 times per second for 30 seconds
time_array = np.zeros(30 * 30)

start_time = time.time()
index = 0
printed = False

def record_fps(fps):
    global index
    fps_array[index] = fps
    time_array[index] = time.time() - start_time
    index += 1


model ='/root/app/models/edl1_1k_edgetpu.tflite'
tpu_interpreter = tflite.Interpreter(model, experimental_delegates=[
    tflite.load_delegate('libedgetpu.so.1.0')])
#cpu_interpreter = tflite.Interpreter(model)
threshold = 0.1



interpreter = tpu_interpreter
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
width = input_details[0]['shape'][1]
height = input_details[0]['shape'][2]

picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}, raw={"size": (1640,1232)}))
#picam2.configure(picam2.create_video_configuration(main={"format": 'RGB888'}))
picam2.set_controls({"AwbMode": controls.AwbModeEnum.Indoor})
#picam2.set_controls({"AwbEnable": 0})
#picam2.set_controls({"ColourGains": (1.3, 1.3)})
picam2.start()
frame_raw = picam2.capture_array()


print(output_details)
frame_count = 0
period_timer_start = time.time()
period_timer_end = time.time()
fps = 0
perf_timer_start = time.perf_counter()
perf_timer_end = time.perf_counter()
reformat_time = 0
        








        
while(True):
    frame = picam2.capture_array()
    frame_count += 1
    period_timer_end = time.time()
    timer_period = period_timer_end - period_timer_start
    fps = 1 / timer_period
    period_timer_start = time.time()


            
            
 
    input_data = np.expand_dims(frame, axis=0)

 



    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    boxes = interpreter.get_tensor(output_details[1]['index'])[0]
    #classes = interpreter.get_tensor(output_details[3]['index'])[0]
    scores = interpreter.get_tensor(output_details[0]['index'])[0]




    record_fps(fps)
    if time.time() > start_time + 30 and not printed:
        df = pd.DataFrame({'Time': time_array, 'FPS': fps_array})
        df.to_csv('/root/logs/fps_data_edl1.csv', index=False)
        printed = True
        print("done")

    
    

    for i in range(len(scores)):
        if ((scores[i] > threshold) and (scores[i] <= 1.0)):

            x1, x2, y1, y2 = int(boxes[i][1] * width) , int(boxes[i][3] * width), int(boxes[i][0] * height), int(boxes[i][2] * height)
            w, h = x2 - x1, y2 - y1
            cx, cy = (int(x1 + 0.5*w),int(y1+0.5*h))
            box_diagonal_length = int(np.sqrt(w**2 + h**2))
            cv2.line(frame, (cx,cy), (cx,cy), (0,255,0), 6)
            cv2.rectangle(frame, (x1,y1), (x2,y2),(255,0,0), 3)
            cv2.putText(frame, "{:.1f}%".format(scores[i]*100), (x1,y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,130,70), 2)

            #print("Inference time", interpreter_elapsed_time)
    cv2.putText(frame,"FPS: {:.1f}".format(fps), (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2 )
    #cv2.imshow('live', frame)
    if (cv2.waitKey(1) == ord('q')):
        break

            # if (len(scores) != 0):
            #     if ((scores[0] > threshold) and (scores[0] <= 1.0)):
            #         x1, x2, y1, y2 = int(boxes[0][1] * raw_width) , int(boxes[0][3] * raw_width), int(boxes[0][0] * raw_height), int(boxes[0][2] * raw_height)
            #         w, h = x2 - x1, y2 - y1
            #         cx, cy = (int(x1 + 0.5*w),int(y1+0.5*h))
            #         box_diagonal_length = int(np.sqrt(w**2 + h**2))


picam2.stop()
cv2.destroyAllWindows()
