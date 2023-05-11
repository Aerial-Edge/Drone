def process_image(self, msg): # callback function main processing function
        
        start_time = time.time()
        self.get_logger().info("Frame Recived")
        dist_and_pos = Int32MultiArray()  #Distance on[0] and x pos on[1] and y pos on[2]
        dist_and_pos = [0,0,0]    
   
        # Function to calculate distance from the object based on its radius in pixels
        def kalkulerDistanse(ballRadius_px):
            return int(faktor / ballRadius_px)
        
        def display_object_info(frame, x, y, radius, distance, color, text_offset):
            if x is not None and y is not None:
                # Draw a circle around the detected object
                cv.circle(frame, (x, y), radius, color, 2)
                
                # Prepare the coordinate text
                coordinates_text = f"X: {x}, Y: {y}"
                
                # Prepare the distance text
                distance_text = f"Distance: {distance} cm"
                
                # Put the coordinate text on the frame
                cv.putText(frame, coordinates_text, (x + 10, y), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Put the distance text on the frame
                cv.putText(frame, distance_text, (22, 70 + text_offset), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Function to detect a colored object within a given color range and size
        def detect_colored_object(colorLower, colorUpper, min_radius, max_radius):
            
            # Create a mask based on the given color range
            mask = cv.inRange(hsv, colorLower, colorUpper)
            
            # Erode the mask to remove noise
            mask = cv.erode(mask, None, iterations=2)
            
            # Dilate the mask to fill gaps
            mask = cv.dilate(mask, None, iterations=2)

            # Convert the frame to grayscale and apply median blur
            gray = cv.cvtColor(opencv_image, cv.COLOR_BGR2GRAY)
            gray = cv.medianBlur(gray, 5)
            
            # Detect circles using the Hough transform
            circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, 20, param1=100, param2=30,
                                    minRadius=min_radius, maxRadius=max_radius)

            # Check if any circles were detected
            if circles is not None:
                circles = np.uint16(np.around(circles))

                for circle in circles[0, :]:
                    x, y, radius = circle

                    # Check if the circle's center is within the mask's boundaries
                    if 0 <= x < mask.shape[1] and 0 <= y < mask.shape[0] and mask[y, x] > 0:
                        return (x, y, radius)
            
            
            return None
        
        # Distance measurement parameters:
        ballRadius = 3.25   # cm (radius of the ball)
        cameraFOV = 62.2    # degrees (field of view of the camera)
        faktor = (1280 / 2) * (ballRadius / math.tan(math.radians(cameraFOV / 2)))

        # Color detection settings:
        colors = {
            'green': {
                'lower': (72, 70, 32), #(L-H, L-S, L-V)
                'upper': (99, 244, 107), #(U-H,  U-S, u-V)
                'min_radius': 0, #ex between 20 
                'max_radius': 0, #to 60 pixels
                'color': (0, 255, 0), #Color of the circle around object
                'text_offset': 0, #Distance text position under FPS
            },
            'orange': {
                'lower': (0, 115, 99), #(L-H, L-S, L-V)
                'upper': (18, 255, 255), #(U-H,  U-S, u-V)
                'min_radius': 0,
                'max_radius': 0,
                'color': (0, 102, 255), #Color of the circle around object
                'text_offset': 20, #Distance text position under FPS
            },
            'red': {
                'lower': (119, 37, 0), #(L-H, L-S, L-V)
                'upper': (179, 179, 147), #(U-H,  U-S, u-V)
                'min_radius': 0,
                'max_radius': 0,
                'color': (0, 0, 255), #Color of the circle around object
                'text_offset': 40,  #Distance text position under FPS
            },
        }

        #fpsreader = FPS() # Initialize FPS reader
        
        # Main loop
       
        #(grabbed, frame) = videoCap.read() # Read a frame from the video capture
        opencv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # convert the ROS message to an OpenCV image
        #fps, opencv_image = fpsreader.update(opencv_image, color=(255, 0, 0)) # Update the FPS overlay on the frame
        #frame = imutils.resize(frame, width=1280) # Resize the frame
        hsv = cv.cvtColor(opencv_image, cv.COLOR_BGR2HSV) # Convert the frame to HSV format

        # Initialize x, y, and ballRadius_px values for each color
        for color_info in colors.values():
            color_info['x'] = None
            color_info['y'] = None
            color_info['ballRadius_px'] = None

        # Iterate through the defined colors and detect objects
        for idx, (color_name, color_info) in enumerate(colors.items()):
            # Call the detect_colored_object function to find objects in the frame
            obj = detect_colored_object(color_info['lower'], color_info['upper'], color_info['min_radius'],
                                        color_info['max_radius'])

            if obj: # If an object is detected
                x, y, ballRadius_px = obj # Get the coordinates and radius of the object
                dist = kalkulerDistanse(ballRadius_px) # Calculate the distance to the object
                #self.get_logger().info(f" FPS : {fps}")

                self.publish_dist_and_pos(x, y, dist)
                self.get_logger().info(f" X = {x}, Y = {y}, Distance = {dist}")

                # Update the color_info dictionary with the new values
                color_info['x'] = x
                color_info['y'] = y
                color_info['ballRadius_px'] = ballRadius_px
                color_info['distance'] = dist

                display_object_info(opencv_image, color_info['x'], color_info['y'], color_info['ballRadius_px'],
                                    color_info.get('distance'), color_info['color'], color_info['text_offset'])
        computaiton_time = time.time() - start_time
        self.get_logger().info("Time to compute: " + str(computaiton_time))
        cv.imshow("Frame", opencv_image)