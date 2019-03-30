import time
import threading


# Image capture thread
class ImageCapture(threading.Thread):
    def __init__(self, camera, processor):
        self.running = False
        self.processor = processor
        self.camera = camera
        super(ImageCapture, self).__init__()
        self.start()

    def run(self):
        print('Start the stream using the video port')
        self.running = True
        self.camera.capture_sequence(
            self.TriggerStream(),
            format='bgr',
            use_video_port=True
        )
        print('Terminating camera processing...')
        self.processor.terminated = True
        self.processor.join()
        print('Processing terminated.')

    def stop(self):
        """ Stop this threaded class """
        self.running = False

    # Stream delegation loop
    def TriggerStream(self):
        """ Method called which loops endlessly """
        while self.running:
            if self.processor.event.is_set():
                time.sleep(0.01)
            else:
                yield self.processor.stream
                self.processor.event.set()
