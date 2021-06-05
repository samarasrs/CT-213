from tensorflow.keras.models import load_model
import cv2
import numpy as np
from utils import sigmoid


class YoloDetector:
    """
    Represents an object detector for robot soccer based on the YOLO algorithm.
    """
    def __init__(self, model_name, anchor_box_ball=(5, 5), anchor_box_post=(2, 5)):
        """
        Constructs an object detector for robot soccer based on the YOLO algorithm.

        :param model_name: name of the neural network model which will be loaded.
        :type model_name: str.
        :param anchor_box_ball: dimensions of the anchor box used for the ball.
        :type anchor_box_ball: bidimensional tuple.
        :param anchor_box_post: dimensions of the anchor box used for the goal post.
        :type anchor_box_post: bidimensional tuple.
        """
        self.network = load_model(model_name + '.hdf5')
        self.network.summary()  # prints the neural network summary
        self.anchor_box_ball = anchor_box_ball
        self.anchor_box_post = anchor_box_post

    def detect(self, image):
        """
        Detects robot soccer's objects given the robot's camera image.

        :param image: image from the robot camera in 640x480 resolution and RGB color space.
        :type image: OpenCV's image.
        :return: (ball_detection, post1_detection, post2_detection), where each detection is given
                by a 5-dimensional tuple: (probability, x, y, width, height).
        :rtype: 3-dimensional tuple of 5-dimensional tuples.
        """

        preprocessed_image = self.preprocess_image(image)
        output = self.network.predict(preprocessed_image)
        ball_detection, post1_detection, post2_detection = self.process_yolo_output(output)
        return ball_detection, post1_detection, post2_detection

    def preprocess_image(self, image):
        """
        Preprocesses the camera image to adapt it to the neural network.

        :param image: image from the robot camera in 640x480 resolution and RGB color space.
        :type image: OpenCV's image.
        :return: image suitable for use in the neural network.
        :rtype: NumPy 4-dimensional array with dimensions (1, 120, 160, 3).
        """
        width = 160
        height = 120

        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
        image = np.array(image)
        image = image / 255.0
        image = np.reshape(image, (1, 120, 160, 3))

        return image

    def process_yolo_output(self, output):
        """
        Processes the neural network's output to yield the detections.

        :param output: neural network's output.
        :type output: NumPy 4-dimensional array with dimensions (1, 15, 20, 10).
        :return: (ball_detection, post1_detection, post2_detection), where each detection is given
                by a 5-dimensional tuple: (probability, x, y, width, height).
        :rtype: 3-dimensional tuple of 5-dimensional tuples.
        """
        coord_scale = 4 * 8  # coordinate scale used for computing the x and y coordinates of the BB's center
        bb_scale = 640  # bounding box scale used for computing width and height
        output = np.reshape(output, (15, 20, 10))  # reshaping to remove the first dimension

        # Todo: implement YOLO logic

        tb = output[0][0][0]
        tib = 0
        tjb = 0

        tp1 = output[0][0][5]
        tip1 = 0
        tjp1 = 0

        tp2 = output[0][0][5]
        tip2 = 0
        tjp2 = 0

        for i in range(15):
            for j in range(20):
                if output[i][j][0] > tb:
                    tb = output[i][j][0]
                    tjb = j
                    tib = i
                if output[i][j][5] > tp1:
                    tp2 = tp1
                    tip2 = tip1
                    tjp2 = tjp1

                    tp1 = output[i][j][5]
                    tjp1 = j
                    tip1 = i
                elif output[i][j][5] > tp2:
                    tp2 = output[i][j][5]
                    tjp2 = j
                    tip2 = i

        txb = output[tib][tjb][1]
        tyb = output[tib][tjb][2]
        twb = output[tib][tjb][3]
        thb = output[tib][tjb][4]

        txp1 = output[tip1][tjp1][6]
        typ1 = output[tip1][tjp1][7]
        twp1 = output[tip1][tjp1][8]
        thp1 = output[tip1][tjp1][9]

        txp2 = output[tip2][tjp2][6]
        typ2 = output[tip2][tjp2][7]
        twp2 = output[tip2][tjp2][8]
        thp2 = output[tip2][tjp2][9]

        pwb = 5
        phb = 5

        pwp = 2
        php = 5

        pb = sigmoid(tb)
        xb = (tjb + sigmoid(txb)) * coord_scale
        yb = (tib + sigmoid(tyb)) * coord_scale
        wb = bb_scale * pwb * np.exp(twb)
        hb = bb_scale * phb * np.exp(thb)

        pp1 = sigmoid(tp1)
        xp1 = (tjp1 + sigmoid(txp1)) * coord_scale
        yp1 = (tip1 + sigmoid(typ1)) * coord_scale
        wp1 = bb_scale * pwp * np.exp(twp1)
        hp1 = bb_scale * php * np.exp(thp1)

        pp2 = sigmoid(tp2)
        xp2 = (tjp2 + sigmoid(txp2)) * coord_scale
        yp2 = (tip2 + sigmoid(typ2)) * coord_scale
        wp2 = bb_scale * pwp * np.exp(twp2)
        hp2 = bb_scale * php * np.exp(thp2)

        ball_detection = (pb, xb, yb, wb, hb)
        post1_detection = (pp1, xp1, yp1, wp1, hp1)
        post2_detection = (pp2, xp2, yp2, wp2, hp2)

        return ball_detection, post1_detection, post2_detection
