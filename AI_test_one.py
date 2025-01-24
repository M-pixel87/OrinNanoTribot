import jetson.inference
import jetson.utils

net = jetson.inference.detectNet(model="/home/uafs/Downloads/jetson-inference/python/training/detection/ssd/models/test_aone/ssd-mobilenet.onnx",
                                 labels="/home/uafs/Downloads/jetson-inference/python/training/detection/ssd/models/test_aone/labels.txt",
                                 input_blob="input_0",
                                 output_cvg="scores",
                                 output_bbox="boxes",
                                 threshold=0.5)

# Initialize camera input and video output
camera = jetson.utils.videoSource("/dev/video0")  # Camera input
display = jetson.utils.videoOutput()  # Display output

while True:
    img = camera.Capture()
    detections = net.Detect(img)
    display.Render(img)

    width = img.width
    height = img.height

    if detections:
        for detect in detections:
            ID = detect.ClassID
            top = int(detect.Top)
            left = int(detect.Left)
            bottom = int(detect.Bottom)
            right = int(detect.Right)
            item = net.GetClassDesc(ID)
            w = right - left
            print(f'Width of object: {w}')  # Print error value for debugging


    display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))
