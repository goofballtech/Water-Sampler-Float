# Water-Sampler-Float

TODO list:

UI:
-Calibrate button for the surface of the water in UI
-From UI ability to clear, reset, download, start profile
-Ability to update time stamps from interface so the float and device are synced for outputs (maybe gps time in future?)
-Battery monitoring?
-Float serial number/name reporting for multiples when needed?
-Make a log txt file and write to the memory for retrieval via web ui when diagnostics are required. Flag info vs error when writing and write to a var so the UI can show if new errors occured during run

ESP32:
-Volume calcs for the buoyancy engine
-Finish status led calcs based on time-time rather than delays
-Adding accelerometers for motion? This would also potentially allow emergency ascend if stationary for longer than a timeout. 
-Bottom detection based on depth change over time? PID control?
-Calculate pressure from the top but offset so depth is from the bottom so when you touch the mud the water depth is the actual mud line
-LoRa deeper research for the comms path, potential LoRa mesh in future to extend them out
-battery charging via inductive or solar or both. 
-How to isolate batteries or not based on charge status, possibly tie to power switch via DPDT type mechanism. 
-LED light status for battery status while on charge?