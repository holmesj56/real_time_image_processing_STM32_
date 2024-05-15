# real_time_image_processing_STM32_

The objective of this project is to apply simple real time image enhancement techniques to live video
stream captured from a webcam.
  • the process of capturing image frames from webcam using MATLAB, sending them through
  serial port to board to process, and receiving the processed frames in MATLAB to display and
  compare
  • coding image processing algorithms including quantization, thresholding, gray level
  transformations and histogram equalization
II. Hardware Setup
• STM32F407G-DISC1 kit
• To establish serial communication the STM32F407 board header pins PA2, PA3, GND, and the
serial/USB converter:
• Webcam:
III. Software Setup
MATLAB is needed to capture video frames, establish serial communication between computer and
board, send and receive video frames, and plot them for comparison.

1. Thresholding
Thresholding, also known as segmentation, aims to divide the image into a foreground and a background.
It can also be used to divide the image into a region of interest and background. Three methods will be
used:
• Global thresholding
• Band thresholding
• Semi thresholding
1) Global thresholding:
This is the simplest segmentation algorithm. An image is reduced to two levels using the following
equation:
𝑓(𝑥, 𝑦) = 255 (𝑤hite) if 𝑝(𝑥, 𝑦) ≥ 𝑇
0 (𝑏lack) if 𝑝(𝑥, 𝑦) < 𝑇
As shown in the equation, all pixels less than the threshold T are set to black (intensity value=0) and all
pixels equal to or above the threshold are set to white (intensity value=255). To choose a suitable
threshold 
2) Band thresholding
Most of the time when the histogram cannot be clearly separated into two regions, one may want to
use what is called “Band Thresholding” technique, where:
𝑓(𝑥, 𝑦) = 255 (white) if 𝑝(𝑥, 𝑦) ∈ 𝐷
0 (black) 𝑜therwise
Band thresholding will segment an image into regions of pixels with gray levels from a set D and into
background. The set D chooses the “region of interest”. This method can be used to generate a mask
corresponding to the region of interest. For this project, the following equation will be implemented:

𝑓(𝑥, 𝑦) = 255 (𝑤hite) if 𝑇1 ≤ 𝑝(𝑥, 𝑦) ≤ 𝑇2
0 (black) otherwise
3) Semi thresholding
Semi thresholding assigns a grayscale value 0 (black) to the background. For a given threshold T, the
equation is:
𝑓(𝑥, 𝑦) =  𝑝(𝑥, 𝑦) if 𝑝(𝑥, 𝑦) ≥ 𝑇
0 (black) if 𝑝(𝑥, 𝑦) < 𝑇
